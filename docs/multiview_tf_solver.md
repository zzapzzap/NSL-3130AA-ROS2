# Multiview TF Solver - 결정론적 멀티태그 체인

NSL-3130 여러 대를 보이는 STag id들로 하나의 공통 `stag_marker` 프레임에 묶는 방식이다.
현재 호스트 solver는 일부러 결정론적으로 동작한다. 관측이 들어온 뒤에는 전역 번들 조정,
삼각측량, LiDAR depth gate를 돌리지 않는다.

목표는 결과를 눈으로 추적 가능하게 만드는 것이다. 로그만 보면 어떤 태그가 원점이 되었는지,
어떤 태그가 각 카메라를 붙였는지, 새로 추가된 태그가 무엇인지, depth가 얼마나 당겨졌는지 확인할 수 있다.

---

## 1. 핵심 규칙

- 앵커 우선순위: `id 0`이 최우선이고, 그 뒤는 id 오름차순이다. `--ref-id`로 선호 앵커를 바꿀 수 있지만 기본값은 `0`이다.
- 카메라 우선순위: 이미 배치된 태그와 공유 태그가 있는 카메라 중, 검출 태그 수가 많은 카메라를 먼저 붙인다.
- 링크 태그 우선순위: 공유 태그가 여러 개면 가장 낮은 id의 공유 태그를 link로 쓴다.
- 체인만 수행: 한 카메라가 배치되면 그 카메라가 새로 본 태그들을 공유 태그 트리에 추가한다. 이후 카메라는 그 트리에 다시 붙는다. 두 번째 최적화 단계는 없다.

즉 `anchor tag -> camera -> new tags -> next camera` 형태로 이어지는 체인 모델이다.

---

## 2. Depth 모델

RGB STag pose는 회전과 bearing은 믿을 만하지만, depth는 약할 수 있다. 그래서 depth는 두 단계에서 다룬다.

1. Edge `multiview_calib_node.py`
   - 각 검출 태그에 대해 카메라 ray를 따라 live LiDAR cloud를 sliding scan한다.
   - marker-frame crop 안에서 가장 조밀한 위치를 고른다.
   - 그 위치에서 1-D RANSAC/plane correction으로 depth를 한 번 더 보정한다.
   - 큰 correction은 기본적으로 버리지 않는다:
     `--max-depth-delta 0.0`, `--slide-search-radius 0.0`,
     `--min-plane-inlier-ratio 0.0`.

2. Host `multiview_solver_node.py`
   - 연결 태그의 LiDAR 보정 range가 해당 카메라의 depth shift 하나를 정한다.
   - 그 shift를 연결 태그 ray 방향으로 카메라 안의 모든 태그에 강체처럼 같이 적용한다.
   - 최종 체인에서는 태그들이 서로 따로 sliding하지 않는다.
   - 나중에 호스트 쪽 카메라 프레임 point cloud를 넣게 되면 `_depth_vote_rigid`가 모든 태그 주변의
     50 cm x 50 cm x 10 cm 박스 inlier 투표로 같은 강체 shift를 고를 수 있다.

기존 방식과 가장 중요한 차이는 이것이다. 53번 카메라처럼 depth가 크게 당겨져야 하는 경우,
그 correction을 조용히 gate로 버리는 대신 체인 로그에 `shift=...m`로 드러낸다.

---

## 3. 프로토콜

```
mtf -> /fleet/calibrate
          |
          v
EDGE cam_51/52/53 :: OBSERVE
  1. STag marker 검출
  2. 태그별 IPPE pose 추정
  3. N개 good view를 median-average
  4. 태그별 LiDAR sliding/RANSAC depth refine
  5. /cam_NN/tag_observations 로 JSON 발행

          |
          v
HOST :: SOLVE
  1. latched observation 수집
  2. anchor tag 선택
  3. 태그 수와 shared-tag 우선순위로 카메라를 greedy하게 배치
  4. --w-up > 0이면 수평 tag normal scoring으로 IPPE flip 결정
  5. 연결 태그 depth shift를 해당 카메라의 tag cluster 전체에 강체 적용
  6. 카메라별 multiview.yml 작성

          |
          v
WRITEBACK
  Host가 각 solved multiview.yml을 /cam_NN/multiview/put 으로 전송
  Edge가 파일을 설치하고 multiview_tf_node가 /tf_static 재발행
```

---

## 4. 출력

작성되는 `multiview.yml`은 기존 포맷을 유지한다.

- `x_cam = R * x_marker + t`
- `marker_id`는 전역 anchor tag
- 해당 카메라가 본 모든 태그를 `tag_k_*` 항목으로 기록
- `bundle_solved: 1`은 호환성을 위해 유지
- `chain_solved: 1`은 결정론적 체인 결과임을 표시

따라서 `multiview_tf_node.py`, configs, writeback service는 포맷 변경 없이 그대로 소비한다.

---

## 5. Solver 진단 로그

`mvw` 터미널의 `[mv_solver]` 블록을 보면 된다.

| 로그 | 의미 |
|---|---|
| `deterministic-chain anchor=id...` | `stag_marker` 원점으로 사용한 태그 |
| `id...: cam..., cam... (bridge)` | 어떤 태그 id가 어떤 카메라들을 연결하는지 |
| `chain CAM...: link=id...` | 이 카메라를 기존 트리에 붙인 태그 |
| `shared=[...]` | 이 카메라가 본 태그 중 이미 배치되어 있던 태그 |
| `added=[...]` | 이 카메라가 새로 트리에 추가한 태그 |
| `depth=link_lidar shift=+...m` | 연결 태그의 edge LiDAR range로 적용한 강체 depth shift |
| `flips=[...]` | up-normal scoring으로 선택된 IPPE alternate |
| `ISOLATED cameras` | anchor까지 shared-tag 경로가 없어 붙지 못한 카메라 |

53번 카메라를 볼 때는 먼저 `chain CAM53` 줄을 보면 된다. 여기서 link id, shift, isolated 여부,
예상과 다른 태그로 연결됐는지를 확인한다.

---

## 6. 주요 인자

Host solver 인자:

| 인자 | 기본값 | 용도 |
|---|---:|---|
| `--ref-id` | `0` | 선호 anchor id |
| `--w-up` | `2.0` | 수평 태그 normal scoring으로 IPPE flip을 결정론적으로 고른다. 태그가 수평이 아니면 `0` |
| `--depth-vote-range` | `0.60` | host cloud vote를 쓸 때 탐색할 연결 ray 반경 |
| `--depth-vote-step` | `0.01` | host cloud vote scan 간격 |
| `--depth-vote-perp` | `0.25` | 수직 방향 반폭. `0.25`면 50 cm box |
| `--depth-vote-half` | `0.05` | ray 방향 반두께. `0.05`면 10 cm box |

예전 BA 튜닝 인자였던 `--w-lidar`, `--lidar-gate`, `--w-depth`,
`--rot-angle-pow`, `--triangulate`는 기존 alias가 깨지지 않도록 받기만 하는 호환용 no-op이다.

Edge depth refine:

| 인자 | 기본값 | 용도 |
|---|---:|---|
| `--depth-band` | `0.05` | RANSAC/refinement crop의 marker plane normal 방향 반두께 |
| `--slide-search-radius` | `0.0` | `0`이면 positive cloud range 전체를 탐색. 양수면 RGB depth 주변으로 제한 |
| `--max-depth-delta` | `0.0` | `0`이면 큰 depth correction rejection 비활성화 |
| `--min-plane-inlier-ratio` | `0.0` | `0`이면 절대 inlier count만 확인 |
| `--slide-crop-x` | `0.35` | marker-frame x 방향 ROI 반폭 |
| `--slide-crop-y` | `0.35` | marker-frame y 방향 ROI 반폭 |
| `--slide-z-band` | `0.03` | marker plane normal 방향 ROI 반두께 |
| `--debug-roi` | `false` | tag별 sliding ROI 박스와 선택된 LiDAR 점을 RViz 토픽으로 발행 |
| `--debug-roi-max-points` | `3000` | tag별 ROI debug point 최대 개수 |

ROI 디버그를 켜면 edge calibration listener가 저장 시점의 snapshot을 latched topic으로 남긴다.

- `/cam_NN/multiview_debug/roi_markers`
- `/cam_NN/multiview_debug/roi_points`

edge 서비스에서 켤 때는 해당 edge의 `~/.ros/nsl_runtime.env`에
`export NSL_CALIB_ARGS="--debug-roi true"`를 넣고 서비스를 재시작한다.

`mvw`의 각 카메라 그룹에는 위 토픽 display가 기본으로 들어간다. 파란 박스는 RGB monocular pose,
주황 박스는 sliding window가 고른 depth, 초록/빨강 박스는 RANSAC refine 결과다.
노란 점들은 실제 선택된 3D ROI 안의 LiDAR 점이다. id0 link가 사람/바닥/배경을 잡으면 여기서 바로 보인다.

Color point cloud 표시는 기본적으로 live 스트림이 아니라 `mtf` 스냅샷 방식이다.

- `mvw`는 기본으로 `/cam_NN/multiview_debug/color_cloud_snapshot`을 본다.
- 각 edge의 `cloud_snapshot_node.py`가 `/fleet/calibrate`를 받으면 최신 RGB point cloud 한 프레임만 발행한다.
- RViz display는 `Decay Time: 30`으로 설정되어 있어 `mtf` 이후 30초가 지나거나 `mvw`를 다시 켜면 오래된
  color cloud가 남지 않는다.
- 계속 live color cloud를 보고 싶으면 `mvw cloud_display_mode:=live`로 실행한다.

---

## 7. 코드 진입점

- Edge 관측/depth: `multiview_calib_node._refine_depth`,
  `multiview_calib_node._publish_observations`
- Host chain: `multiview_solver_node.BundleSolver._greedy_chain`
- 강체 depth vote helper: `multiview_solver_node.BundleSolver._depth_vote_rigid`
- Writeback: host `_push`, edge `multiview_put_server.py`
- 검증: `python3 multiview_solver_node.py --selftest`
