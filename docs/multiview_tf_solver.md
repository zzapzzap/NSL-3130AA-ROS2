# Multiview TF Solver — 멀티태그 번들 재캘리브레이션

NSL-3130 카메라 여러 대를 **하나의 공통 `stag_marker` 프레임**으로 묶어 RViz / 포즈 스택이 모든 뷰를
융합할 수 있게 하는 방식. 단일 레퍼런스 태그에 의존하던 기존 방식을, **어떤 태그도 모든 뷰에서 보일
필요가 없는 멀티태그 번들 조정(bundle adjustment)** 으로 교체한 것.

---

## 1. 왜 만들었나

기존 `multiview_calib`는 **하나의** 레퍼런스 태그(id 7)가 **모든** 카메라에 보여야 했다:

- 어떤 카메라가 id 7을 못 보면, 노드가 조용히 "가장 많이 본 태그"로 폴백 → 카메라마다 **다른 물리
  태그**를 `stag_marker`라 부르며 어긋남(경고도 없이).
- 보조 태그들은 저장만 되고 **융합되지 않음** — 각 카메라가 자기 레퍼런스 관측 하나로 태그를 찍을 뿐,
  카메라 간 합의가 없었다.
- 바닥/태그 주변 깊이를 1-D ray slide + 1-D RANSAC로 잡아서 엉뚱한 표면에 lock 걸리기 쉬웠다.

해결: 태그를 여러 개(예: id 0–3, 높이 제각각/계단식) 흩뿌리고, **두 카메라가 같이 보는 태그가 둘을
잇는 "브릿지"** 역할을 하게 한 뒤, **모든 카메라·태그 포즈를 한꺼번에** 푼다. 단일 태그가 모두에게
보일 필요 없이, **태그-가시성 그래프가 연결만 되어 있으면** 된다.

---

## 2. 신뢰 모델 (캘리브레이션 규칙)

평면 마커의 RGB 포즈는 **방향(heading·bearing)은 정확하지만 거리(depth)는 약하다**. 솔버는 이걸 그대로
인코딩한다:

| 양 | 출처 | 취급 |
|---|---|---|
| 회전 / heading | STag 코너 (PnP) | **신뢰** — 단, 비스듬한(grazing) 뷰는 틸트가 부정확해 `rot_angle_pow`로 완화 |
| bearing (이미지 내 방향) | STag 코너 | **신뢰** — 그래프를 끌고 가는 1차 신호 (`w_lat` 최강) |
| 절대 스케일 | 태그 실측 크기 | **RGB만으로 확정** — LiDAR 불필요 |
| ray 방향 깊이 | LiDAR sliding+RANSAC | **깊이 미세보정**: LiDAR 보정 거리가 각 태그 거리를 끌어당김(`w_lidar`), **평면 confidence + RGB-only gate**로 태그별 가중/거절 |
| 모노큘러 깊이 | 태그 크기 PnP | **약한 fallback(`w_depth`)** — LiDAR 평면이 없는 태그에만. `w_lidar`보다 낮게 둬서 LiDAR가 깊이를 주도 |

- **GICP(클라우드 ICP) 안 씀**: 기하 중첩이 필요한데 뷰가 sparse하면 발산. 태그는 엔지니어드 대응점이라
  **두 카메라가 태그 하나만 공유**(또는 체인 연결)하면 브릿지가 된다.
- **단일 바닥평면 가정 안 씀**: 태그가 **계단식(높이 제각각)** 이라 태그마다 6DoF 자유. 깊이는 LiDAR
  prior + 브릿지 삼각측량이 정한다.

### 메인 태그 우선순위

앵커(원점)와 init 순서는 **id 7(큰 0.32 m 태그) → id 0 → id 1 → id 2 → …** 우선순위를 따른다
(= `ref_id`가 보이면 그걸, 없으면 가장 낮은 id). 이렇게 해야 카메라마다 다른 메인을 잡는 일이 없고,
고우선(크고 신뢰도 높은) 태그부터 TF 백본을 깔아 **뒤틀림을 막는다**. `--ref-id`로 최우선 태그 변경 가능.

---

## 3. 프로토콜 — 3단계

```
mtf  ─────▶  /fleet/calibrate (std_msgs/Empty 방송)
                 │
   ┌─────────────┼─────────────┐   (엣지 병렬)
   ▼             ▼             ▼
╔══ EDGE cam_51/52/53 :: (1) OBSERVE  [multiview_calib_node.py] ══╗
║ 1. STag(HD21) 검출                                              ║
║ 2. 태그별 solvePnP(IPPE_SQUARE) → R,t (태그 in 카메라)          ║
║ 3. N뷰 median R/t + reproj rmse + 평균 코너                     ║
║ 4. LiDAR sliding+RANSAC 깊이 보정 (after_m, 인라이어/지지점 수)  ║
║ 5. JSON 패킹 {K,D,fisheye, 태그별: R,t,코너,크기,depth}         ║
║    → /cam_NN/tag_observations (latched) 1회 발행                ║
╚══════════════════════════════════════════════════════════════════╝
                 │  │  │   (카메라당 latched JSON 토픽 1개)
                 ▼  ▼  ▼
╔══ HOST :: (2) SOLVE  [multiview_solver_node.py] ════════════════╗
║ a. settle 윈도우 동안 전 카메라 관측 수집                        ║
║ b. 그래프 구성: 노드=카메라+태그, 엣지=관측 (같은 id=같은 노드)  ║
║ c. 연결성 검사: 앵커서 도달 못한 노드는 경고+제외               ║
║ d. 앵커 태그(우선순위) = 원점 고정, 우선순위·품질 순 spanning init║
║ e. 호스트가 코너에서 양쪽 IPPE 해 복원 → 전역 rms 하강 flip-repair║
║ f. 전역 SE3 BA: bearing(강) + 회전(grazing 완화)                ║
║    + LiDAR 깊이(confidence 가중, 주도) + 모노큘러(약), Huber     ║
║ g. → 카메라별 multiview.yml (기존 _save 포맷 동일)              ║
╚══════════════════════════════════════════════════════════════════╝
                 │  │  │   (각 카메라의 풀린 yml)
                 ▼  ▼  ▼
╔══ (3) WRITEBACK  (호스트 → 각 엣지, 역방향) ════════════════════╗
║ 호스트가 cam_NN의 풀린 multiview.yml을                          ║
║   /cam_NN/multiview/put (청크+sha256, PutWeight 재사용) 으로 전송 ║
║ 엣지가 calib_output/{serial}/multiview.yml 에 원자적 설치        ║
╚══════════════════════════════════════════════════════════════════╝
                 │
                 ▼  (엣지에서 자동)
   multiview_tf_node 가 mtime 변화 감지(1초 폴링)
   → /tf_static 재발행 → 호스트 RViz가 새 정렬로 실시간 반영
```

### writeback이 뭔가

**SOLVE는 호스트**에서 돌지만, 실제로 TF를 쏘는 건 **각 엣지**(fleet-local: 엣지가 자기 프레임 소유).
그래서 호스트가 푼 결과(카메라별 `multiview.yml`)를 **다시 각 엣지로 되돌려 써주는(write back)** 단계가
③. 안 하면 결과가 호스트 메모리에만 있고 `/tf_static`엔 반영이 안 된다.

---

## 4. 번들 조정(BA)

포즈는 `(R, t)` = `x_parent = R·x_child + t`.

- **변수**: 카메라 월드 포즈 `T_w_camᵢ` + 태그 월드 포즈 `T_w_tagⱼ` (각 6DoF). **전역 동시 최적화**라
  카메라를 푸는 *순서*는 수렴값에 영향 없음(순차 처리가 아님).
- **게이지**: 앵커 태그(우선순위 최상)를 원점(`T=I`)에 고정 = `stag_marker`. 스케일은 태그 크기로 확정.
- **초기화**: 앵커에서 BFS spanning tree, **우선순위·품질(큰·정면·낮은 rmse) 순**으로 확장.
- **flip-repair**: 호스트가 코너에서 IPPE 두 평면해를 복원 → 한 관측을 대안 해로 뒤집고 **처음부터
  재최적화**, 전역 rms가 줄면 채택. 틀린 틸트가 그래프 전체를 비틀어 흡수하는 wrong-basin을 전역 rms로
  탈출. ("solvePnP가 엉뚱한 방향" 해결)
- **잔차** (카메라 i가 태그 j 관측, 예측 `T_camᵢ⁻¹∘T_w_tagⱼ` = R_pred,t_pred):
  - 회전: `w_rot·cos(입사각)^pow · log_SO3(R_measᵀ R_pred)` (정면=꽉, grazing=완화)
  - 병진(ray 기준 분해): `√w_lat·수직 + √w_depth·ray방향`
  - LiDAR 깊이: `√(w_lidar·conf)·(‖t_pred‖ − range_lidar)` — **깊이 미세보정**, 평면 confidence로 가중
  - LiDAR gate: 먼저 RGB/브릿지만으로 한 번 풀고, 그 예비해의 태그 거리와 LiDAR 거리가 `--lidar-gate`
    이상 다르면 해당 LiDAR prior를 제거. 53번처럼 한 카메라 depth가 엉뚱한 평면에 붙는 경우 전체 BA를 보호한다.
  - robust loss: Huber
- **솔버**: `scipy.optimize.least_squares`. 3카메라×~5태그면 즉답.

### 출력 / 불변식

카메라별 `multiview.yml`은 `multiview_calib_node._save`와 **완전히 동일한 포맷**(`x_cam = R·x_marker + t`,
`marker_id` = 전역 앵커, 모든 `tag_k_*`)으로 써서 `multiview_tf_node.py`·`configs.py`가 **수정 없이** 소비.
`bundle_solved: 1` 플래그로 전역해 산출물 표시.

---

## 5. 구성요소

| 위치 | 파일 | 역할 |
|---|---|---|
| 엣지 | `roboscan_nsl3130/scripts/multiview_calib_node.py` | OBSERVE — 검출 + `/cam_NN/tag_observations` 발행 |
| 호스트 | `roboscan_nsl3130/scripts/multiview_solver_node.py` | SOLVE — 수집·번들조정·yml 작성·(writeback) |
| 엣지 | `roboscan_nsl3130/scripts/multiview_put_server.py` | WRITEBACK 수신 — `/cam_NN/multiview/put` |
| 엣지 | `roboscan_nsl3130/scripts/multiview_tf_node.py` | `stag_marker → 프레임` 발행, mtime 폴링 재발행 |

토픽/서비스: `/fleet/calibrate`(Empty, 호스트→엣지), `/cam_NN/tag_observations`(String JSON, latched,
엣지→호스트), `/cam_NN/multiview/put`(`PutWeight`, 호스트→엣지). 새 ROS 인터페이스/빌드 변경 없음
(관측=JSON, writeback=PutWeight 재사용).

---

## 6. 실행 / 진단

실행 명령과 튜닝 노브(`mgp`/`mcb`/`mvw`/`mtf`, `solver_args`, `calib_args`)는 **운영 문서 → README 6-7**에
정리돼 있다. 이 문서는 *왜 그렇게 동작하는지*(메커니즘)와 인수인계에 집중한다.

**진단 로그 읽는 법** (`mvw` 터미널의 `[mv_solver]` 블록):

| 줄 | 의미 |
|---|---|
| `anchor=id… (origin)` | 게이지로 고정된 메인 태그(우선순위 7→0→1…). 카메라마다 같아야 정상 |
| `id…: cam_…, cam_… (bridge)` / `★anchor` | 어느 태그가 어느 카메라를 잇는지 = **대응(브릿지) 그래프** |
| `IPPE flip-repaired […]` | 평면 마커의 틀린 틸트를 전역해로 뒤집어 고친 관측 |
| `lidar gate rejected cam_…/id…` | LiDAR depth가 RGB 예비해와 `--lidar-gate` 이상 달라 폐기된 관측 |
| `residual cam_…/id… = … ← HIGH` | 그 대응/깊이가 전역해와 안 맞음 — 불량 검출·미해소 flip·**extrinsic 편향** 의심 |

---

## 7. 인수인계 메모

**설계 결정 (왜 이렇게 했나)**
- *단일 레퍼런스 태그 → 멀티태그 브릿지*: 한 태그가 모든 뷰에 안 보여도 가시성 그래프가 연결되면 풀린다.
  단일 태그 폴백이 카메라마다 다른 메인을 잡아 어긋나던 버그를 제거한 것이 핵심.
- *flip-repair를 전역 rms로*: 평면 마커 IPPE 2중해(틸트)는 **국소 비교로 못 가린다** — BA가 그래프 전체를
  비틀어 틀린 틸트를 흡수하기 때문. 한 관측을 뒤집고 **처음부터 재최적화**해 전역 rms가 줄 때만 채택해야
  wrong-basin을 탈출한다. (단일 35° 오류 주입 → 정확 복구를 `--selftest`가 검증)
- *depth confidence + lidar-gate*: LiDAR 평면이 약하거나(53번) RGB 예비해와 어긋나면 그 깊이 prior를 줄이거나
  버려서, **한 카메라의 나쁜 깊이가 전체 BA를 오염**시키지 않게 한다. RGB가 깊이를 1차로 잡고 LiDAR는 보정.
- *우선순위 앵커(7→0→1…)*: 결정적·안정적 원점 + 신뢰도 높은(큰) 태그부터 TF 백본 → 뒤틀림 감소.

**알려진 한계 / 확장 포인트**
- 한 태그를 **한 카메라만** 보면 그 깊이는 모노큘러뿐 → 태그를 카메라들이 **겹쳐 보게** 깔아 브릿지(≥2뷰) 확보.
- **extrinsic(LiDAR→RGB)이 편향된 카메라**는 솔버가 못 고친다(마커 TF는 맞아도 클라우드가 평행이동/기움)
  → 그 카메라 `extrinsic_calib` 재실행. (SSH로 3대 extrinsic `R`/`t` 비교하면 편향 카메라가 드러남.)
- BA는 `scipy.optimize.least_squares`(소규모 전용). 카메라/태그가 수십 개로 늘면 희소 솔버(ceres/g2o) 고려.
- flip-repair는 다중 flip을 greedy로 처리 — 동시에 여러 개가 얽혀 틀리면 라운드 수↑ 또는 per-tag 합의로 보강.
- **인터페이스 무증설 원칙**: 관측=`std_msgs/String` JSON, writeback=기존 `PutWeight` 재사용 → 새 rosidl/빌드
  변경 0, 엣지 재배포 최소. 바꿀 때 이 원칙을 깨지 않도록.

**코드 진입점**
- 엣지 OBSERVE: `multiview_calib_node._publish_observations` (태그 관측 JSON 발행), `_refine_depth` (LiDAR slide+RANSAC).
- 호스트 SOLVE: `multiview_solver_node.BundleSolver.solve` (그래프·우선순위 init·flip-repair·lidar-gate·BA),
  `_edge_residual` (잔차 정의), `parse_observation` (코너→양쪽 IPPE 해 복원).
- WRITEBACK: 엣지 `multiview_put_server`(수신), 호스트 솔버의 `_push`(전송).
- 검증: `python3 multiview_solver_node.py --selftest` (브릿지·루프·flip-repair·앵커 단위 테스트).
