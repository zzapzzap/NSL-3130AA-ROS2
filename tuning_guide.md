# NSL-3130AA 다중 카메라 튜닝 가이드

목표: **유효 PointCloud를 많이 얻고, Depth 흔들림(fluctuation)·노이즈를 줄이며, 실제 동시 녹화 Hz를 높인다.** 틀린 거리·이전 프레임 재사용으로 점 수나 안정성만 높이는 설정은 채택하지 않는다.

2026-10-05 통합 후 실행 이름은 `cam_161..cam_165`다. 아래 52/53 관측 기록은 기존 bag의 이름을 유지한다. 기존 52→161, 53→162 장비 연결을 이어받았다면 같은 시리얼의 이상을 추적할 수 있지만, 새 장비 시리얼 대응은 실제 연결 후 확인한다.

## 1. 먼저 읽을 실행 요약

**다음 작업자는 우선 1·2·7절만 읽고, 조절할 값은 4절·실행 명령은 6절을 필요한 만큼 참조한다.** 실제 코드가 문서와 다르거나 시험이 막힐 때만 마지막 코드 진입점을 조사한다.

1. 각 Edge 시작 로그의 `sensor params:` 경로와 시리얼을 확인하고 **실제로 선택된 파일을 백업**한다.
2. 고정 장면에서 **53 유지 / 52 방출 OFF → ON → OFF**를 비교한다. 이어서 52도 단독/동시를 비교한다.
3. 프로파일·채널 분리를 확인하고, 간섭 검출을 시험한다. 거리 편향이 남은 상태에서 TF나 배율로 덮지 않는다.
4. **한 번에 한 항목, 같은 장면에서 각 30초**: 진폭 → 공간/시간 필터 → 노출/HDR → 녹화 부하 순으로 비교한다.
5. 매 시험에서 `유효 점/프레임 · 거리 오차 · Depth 흔들림 · 수신/녹화 Hz · 유효 점/초`를 함께 기록한다.
6. 최종 카메라 수를 모두 켜고 **정지 장면 + 움직이는 로봇 + 실제 녹화**를 검증한다. 변경 직후 스트림 재시작 구간은 제외한다.
7. 선정 설정을 재시작 후 확인하고, **이 파일의 상태와 결과표를 갱신**한다. 미측정 결과는 확인 완료로 쓰지 않는다.

## 2. 확인된 현상과 진행 상태

아래 관측·상태표는 **2026-10-05 당시 기록**이며 현재 서비스 상태나 이후 시험의 완료 여부를 뜻하지 않는다. 새 시험은 7절의 결과 양식에 별도로 기록한다.

2026-10-05 사용자 반복 관측: **52/53의 상호 정렬은 대체로 맞아 보이지만, 두 대 이상 운용할 때 Depth 이상이 발생한다. 특히 53은 앞 기둥을 맞춰도 뒤 기둥·로봇이 더 멀리 늘어나 보인다.** `1.2배`는 관측 예시이며 고정 보정계수로 확정하지 않았다.

| 항목 | 당시 기록 | 완료 시 남길 근거 |
|---|---|---|
| 다중 카메라 Depth 이상 현상 | **확인됨 — 사용자 반복 관측** | 발생 카메라·장면·bag |
| 원인 | ToF 광학 간섭을 우선 조사; 원인 분리 시험 대기 | 방출 OFF/ON/OFF 결과, 선택 프로파일·채널 |
| 튜닝 실행 | 미실행 | 시험일·변경값·실측표·선정 파일 |
| 3개 요구조건 검증 | 미실행 | 유효 점 수·흔들림·녹화 Hz의 전/후 비교 |
| 동시 녹화 해결 여부 | 검증 대기 | 최종 카메라 수에서 거리·정지/동작·녹화 검증 |

**튜닝 작업자는 시험 후 위 상태를 직접 수정한다.** 실행했으면 `튜닝 실행: 완료`, 요구조건을 실측으로 만족했으면 `요구조건 검증: 확인됨`으로 바꾸고 날짜와 결과 경로를 적는다. 단순 파일 변경, 채널 번호 분리, RViz 정렬만으로 `해결됨`이라고 쓰지 않는다. 한 항목이 악화되면 `부분 개선`과 채택한 절충을 기록한다.

### 기존 설정에서 확인한 사항

- 일반 실행은 공통 `config/config.yaml`의 `parameters`가 가리키는 프로필을 우선 사용한다. 공통 설정이 없거나 `SIRIUS_CONFIG`로 모듈 설정을 명시하면 `setup/fleet.yaml`을 사용한다. 선택 폴더에 센서 설정이 없으면 실제 시리얼의 `calib_output/{serial}/params.yaml`을 읽고, 새 프로필은 공통 general 파일로 초기화한다. **`calibration:=true`는 공통 calibration 파일(Channel 0)을 선택**한다. 기본 Ethernet 모드에서는 장치 identity 조회 실패가 시작 오류이며, 시리얼·프로필이 모두 없는 단독 실행의 general 기본값과 구분한다. namespace `cam_161`/`cam_162`은 장비 시리얼과 별개이므로 시작 로그의 `sensor params:` 경로를 확인한다.
- 현재 저장된 시리얼 프로필 기준: `N010005C4`는 Channel 1 / MinAmplitude 10 / `HDR_None` / int0 1000, `N008B060D`는 Channel 0 / MinAmplitude 5 / `HDR_None` / int0 350이다. 둘 다 int1/int2는 700/80이며 `MOD_12Mhz`, `DB_6Mhz`, `DB_CORRECTION`, `InterferenceDetectionLimit: 0`, `UseLastValue: false`, Median/Gaussian ON, Temporal 300/Threshold 200이다. **이는 파일 내용이며 장치 적용·거리 정확도를 확인한 최적 프리셋이 아니다.** 장비를 교환하면 실제 시리얼과 선택 파일부터 재확인한다.
- 간섭 검출 `0`은 OFF. 채널은 이미 0/1로 다르지만 SDK 자동 채널 상태는 ROS/YAML에 노출되지 않고, 적용 후 실제 채널 readback도 없다. ROS 파라미터 조회는 요청/캐시값이지 장치 적용 성공의 증명이 아니다.
- `DB_CORRECTION`은 phase wrapping 보정이며 카메라 간섭 제거를 보장하지 않는다. 양쪽 보조 6 MHz가 같다는 사실만으로 충돌을 확정하지 않는다.
- RViz 표시 OFF는 적외선 방출 OFF가 아니다. 방출은 제조사가 지원하는 정지 방법 또는 해당 카메라 전원 OFF로 제어한다. 전원 OFF는 통신·전원 부하도 바꾸므로 그 조건을 기록한다. 저장 bag/snapshot은 이 시험에 반응하지 않으므로 **새 라이브 데이터**를 본다.

## 3. 무엇을 측정하고 선정할까

카메라·대상·조명·예열 조건을 고정하고 가까운/중간/먼 기준면을 정한다. 유리·금속 반사면만으로 거리 정확도를 판단하지 말고, 무광 기준면과 로봇 장면을 함께 확인한다. Depth는 `0 < d ≤ MaxDistance`인 픽셀만 통계에 쓰고, 제외된 픽셀의 비율을 별도로 기록한다. `MaxDistance≤0`으로 출력 컷을 끈 시험에서는 SDK 무효 코드를 제외한 동일한 분석 거리 범위를 따로 정한다.

| 요구조건/검사 | 측정 기준 |
|---|---|
| PointCloud 많이 | raw `/camera/point_cloud`의 **finite XYZ 유효 점 수** 평균. 같은 시야/ROI/MaxDistance를 유지한다. organized cloud의 `width×height`는 NaN 포함 슬롯 수이므로 사용하지 않는다. |
| 흔들림·노이즈 적게 | 고정 픽셀/작은 ROI의 프레임별 Depth 중앙값 `d(t)`에서 `P95−P5`(mm), 무효 비율, 튀는 점을 기록한다. 움직이는 로봇에서는 잔상·반응 지연도 확인한다. |
| Hz 높게 | 각 Edge와 Host의 raw/RGB cloud 수신 Hz, **녹화 중 Hz와 bag 프레임 수**를 비교한다. RViz FPS는 센서 Hz가 아니다. |
| 거리 정확도 | 가까운/중간/먼 기준점의 실측 거리와 Depth 중앙값 오차(mm). Depth 거리와 비교할 때는 카메라에서 해당 점까지의 시선 방향 거리로 맞춘다. TF 정렬·흔들림 감소만으로 정확도 합격을 판단하지 않는다. |
| 종합 처리량 | `유효 점/초 = 평균 유효 점/프레임 × 실제 녹화 Hz`. RGB cloud는 RGB 시야 밖 점도 제외하므로 raw 점 수와 따로 기록한다. |

정량 목표가 있으면 결과표에 먼저 적는다. 목표가 없으면 **동일 장면의 튜닝 전 동시 녹화 대비 점 수·Hz는 증가/유지, 흔들림은 감소/유지, 거리 오차·잔상은 악화되지 않는 후보**를 우선 선정하고 그중 유효 점/초가 높은 값을 고른다. 상충하면 개선율과 손실을 수치로 적고 `부분 개선`으로 남긴다.

finite XYZ에도 잘못된 거리가 포함될 수 있으므로 거리 오차를 함께 본다. 점 수·흔들림 통계는 진단 bag 분석으로 산출하고, 시각적으로 좋아 보인다는 이유만으로 검증 완료 처리하지 않는다.

## 4. 조절할 값 — YAML과 ROS 이름을 구분

ROS 파라미터 이름은 공백/마침표가 있으므로 명령에서 따옴표로 감싼다. 값 변경은 현재 YAML에 **자동 저장**되고 SDK 장치 저장 함수도 호출되며 스트리밍이 잠시 재시작된다. 3D 경계 필터는 SDK 내부 설정으로 장치에 직접 저장되지 않으며 YAML에서 재적용된다. 별도의 save 명령은 없다.

### 4-1. 거리 측정·유효 점·필터

phase wrapping은 신호 주기를 넘는 먼 거리가 짧은 거리처럼 접혀 측정되는 현상이다. 필터는 점을 제거하거나 평균하고, 노출/HDR는 획득 조건을 바꾸며, 주파수/DualBeam은 거리 해석에 관여한다. 서로 같은 역할이 아니다.

| YAML 키 | ROS/rqt 이름 | 효과·시험 방법 |
|---|---|---|
| `Channel` | `K. channel` | 0–15. 동일 시야 카메라의 채널을 분리하되 0/1이라는 이유만으로 완료 처리하지 않는다. |
| `Modulation` | `J. modIndex` | ToF 거리 계산의 변조주파수. `MOD_24Mhz/12Mhz/6Mhz/3Mhz` 지원. 제조사 기준 기본 비모호 거리 약 6.25/12.5/25/50 m; 높은 주파수는 정확도에 유리하나 거리가 짧아진다. Hz 설정값이 아니며 채널·DualBeam·원거리 대상을 함께 재검증한다. |
| `InterferenceDetectionLimit` | `V. interferenceDetectionLimit` | 0=OFF. 제조사 예시 **400 lsb**부터 시험하고 필요 시 200/800을 각각 비교한다. 거리 mm가 아니다. ROS callback은 1000까지 제한하므로 UI의 10000 표시를 그대로 믿지 않는다. |
| `UseLastValue` | `V. useLastValue` | 간섭 검출 시 이전 픽셀 재사용. 진단은 **false**로 해서 낡은 값으로 안정성을 숨기지 않는다. true 채택 시 움직임/잔상을 별도 검증한다. |
| `MinAmplitude` | `I. minAmplitude` | 거리값을 남길 최소 반사 진폭. 높이면 약한 점을 제거해 점 수 감소/잡음 억제, 낮추면 점 수/잡음 증가. 현재값 주변 후보 5/10/15/20을 순차 비교하고 필요할 때 범위를 넓힌다. **허용 오차·흔들림을 만족하는 가장 낮은 값**을 선정한다. |
| `MedianFilter` | `R. medianFilter` | 고립된 튀는 점 억제 후보. 현재 ON/OFF 한 항목만 비교한다. 얇은 구조 손실 확인. |
| `GaussianFilter` | `S. gaussianFilter` | 공간 평활화. ON/OFF를 비교하고 기둥/로봇 경계 번짐 확인. |
| `TemporalFilterFactor` | `T. temporalFilterFactor` | **YAML 300 = ROS 0.3** = 현재 30% + 이전 70%. 후보 ROS 0.7→0.5→0.3; 작을수록 평활화/지연 증가. YAML 0/1000, ROS 0.0/1.0은 OFF. |
| `TemporalFilterThreshold` | `T. temporalFilterFactorThreshold` | 시간 필터 적용 여부를 정하는 거리 차이(mm), SDK 0–1000. 현재 200이면 이전/현재 픽셀 차이가 200 mm 미만일 때만 평활화. 높이면 더 큰 변화까지 평균해 잔상이 늘 수 있고, 낮추면 변화에 빨리 반응하지만 흔들림이 남을 수 있다. 0=OFF. |
| `EdgeFilterThreshold` | `U. edgeFilterThreshold` | 인접 Depth 차이가 임계값을 넘는 점을 무효 처리. 0=OFF, 1–5000. **양수 범위에서 낮추면 더 엄격**, 높이면 경계 점을 더 남긴다. 얇은 구조/유효 점 손실 비교. |
| `PointColud EDGE` | `Y. PointColud EDGE` | **오타까지 실제 키**. XYZ 기반 3D 경계 필터, 0=OFF, SDK 범위 0–5000. 양수 임계값을 낮추면 경계 제거가 강해질 수 있다. 현재 200과 OFF를 비교하고 점 수·경계 정확도를 함께 본다. |
| `IntegrationTime3D` | `E. int0` | 기본 Depth 노출값(드라이버 범위 0–2000). 높이면 약한 반사 확보에 유리하나 포화/획득 시간 증가 가능, 낮추면 Hz에 유리하나 약한 점 손실/노이즈 증가 가능. 노출 묶음 순서 확인 후 시험. |
| `IntegrationTimeHdr1` | `F. int1` | Spatial/Temporal HDR의 추가 노출. 기본 노출보다 작게 사용해 밝은 대상과 어두운 대상의 측정 범위를 보완한다. 추가 획득 비용과 포화 감소를 비교. |
| `IntegrationTimeHdr2` | `G. int2` | Temporal HDR의 추가 짧은 노출. int1보다 작게, 미사용 시 0. 추가 값이 실제 획득/Hz에 미치는 영향을 확인. |
| `HDRMode` | `D. hdr_mode` | `HDR_None` / `HDR_Spatial` / `HDR_Temporal`. Spatial은 한 프레임 안의 두 노출, Temporal은 여러 노출 프레임으로 결과 생성. 밝기 차이·포화 대응과 Hz/동작 오차가 상충하므로 로봇 움직임도 시험. |
| `DualBeam` | `W. dualBeam` | `DB_Off` / `DB_6Mhz` / `DB_3Mhz`. 보조 주파수로 phase wrapping을 판단/보정한다. OFF는 보조 획득 비용을 줄일 후보지만 원거리 접힘 검사를 먼저 한다. 간섭 제거 스위치가 아니다. |
| `DualBeamOption` | `W. dualBeamOption` | `DB_AVOIDANCE`: 접힌 거리 제거. `DB_CORRECTION`: 접힌 거리 복원. `DB_FULL_CORRECTION`: 약한 진폭 거리까지 포함. 남는 점 수가 많다고 정확도까지 좋은 것은 아니다. |
| `MaxDistance` | `Z. MaxDistance` | 출력 거리 컷(mm), 일부 모드의 표시 색상 범위. 높이면 먼 점을 더 남기고 낮추면 먼 점 제외; ≤0은 드라이버 출력 컷 OFF. **Depth 배율 보정이나 센서 측정 범위 확장이 아니다.** raw Depth는 SDK 거리, cloud는 SDK 전방 OUT_Z 기준으로 컷한다. |
| `ImageType` | `C. imageType` | 출력/획득 모드. `RGB_DISTANCE`는 RGB+Depth, `RGB_DISTANCE_AMPLITUDE`는 진폭 이미지도 발행. 필요한 모드만 사용하고 추가 획득/전송의 Hz 영향 측정. `RGB`/`GRAYSCALE` 단독은 Depth cloud를 발행하지 않는다. |
| `IntegrationTimeGray` | `H. intGr` | Grayscale 노출(드라이버 범위 0–40000). Depth/RGB 기본 노출과 별개이며 Gray 모드에서 의미가 있다. 높이면 밝기·포화/획득 시간이 증가할 수 있다. 현재 100; Gray 획득이 필요할 때만 조정. |
| `GrayscaleLED` | `X. grayscale LED` | Grayscale 획득 중 LED 사용 여부. **Depth 적외선 방출 전체를 끄는 스위치가 아니다.** 간섭 OFF/ON 시험에 이 값을 대신 사용하지 않는다. |

**노출 주의:** SDK 매뉴얼은 `int1 < int0`, `int2 < int1`, 미사용 `int2=0`을 안내한다. 공통 general/calibration 파일과 `N008B060D`에 남은 `350/700/80`은 이 순서와 다르므로 검증된 고속 프리셋으로 추천하지 않는다. `N010005C4`의 저장값은 `1000/700/80`이다. `HDR_None`이어도 추가 노출값이 무시된다고 단정할 수 없다. 장비/SDK 동작과 순서를 확인한 뒤 정합된 노출 묶음을 기준으로 ±20%씩 비교한다. 거리·포화·점 수가 나빠지면 복귀한다. 현재 ROS 드라이버에는 `fps:=30` 같은 Hz 설정 인자가 없다.

### 4-2. 렌즈·좌표·시야·표시 설정

| YAML 키 / ROS 이름 | 역할과 변경 시 주의 |
|---|---|
| `LensType` / `B. lensType` | `LENS_NF/SF/WF`(SDK 화각 50°/90°/110°) 중 **실제 장착 렌즈**에 맞춰 SDK 3D 재구성을 선택. 노이즈·배율 조정용이 아니며 임의 변경은 형상 오류를 만들 수 있다. |
| `LidarAngle` / `P. transformAngle` | SDK에 전달하는 설치 수직각(도). 점 좌표 방향에 영향; 거리 배율 보정이 아니다. ROS callback에서 정수로 잘리며 SDK 재연결이 발생하므로 세밀한 TF 조정은 TF 쪽에서 한다. |
| `Q. frameID` (ROS만) | 메시지 좌표계 이름. 이름을 바꿔도 좌표값은 회전/이동하지 않는다. TF와 일치시켜야 한다. |
| `L. roi_leftX`, `M. roi_topY`, `N. roi_rightX` (ROS만) | SDK가 획득할 픽셀 영역. 축소하면 시야/총 점 수가 줄고 처리량 개선 가능; 같은 전체 시야 요구에서 유리한 튜닝으로 간주하지 않는다. 드라이버가 격자 단위로 보정하며 Y 아래쪽은 위쪽 값에서 대칭 계산한다. 현재 flat YAML에는 ROI가 저장되지 않으므로 재시작 후 직접 확인한다. |
| `A. cvShow` (ROS만) | 드라이버 OpenCV 창 표시. OFF는 표시 부하 감소 후보이며 Depth 품질 설정이 아니다. RViz/rqt는 launch의 `use_rviz`/`use_rqt`로 별도 제어한다. |
| `IsReference` (기존 YAML 일부) | 현재 드라이버에서 읽거나 사용하지 않는 키이며 저장 시 빠진다. 메인 앵커·카메라 동기화·간섭 방지 기능으로 해석하지 않는다. |

### 4-3. 증상에서 바로 찾기

| 증상 | 먼저 볼 파라미터/조건 |
|---|---|
| 점 부족 | `MinAmplitude`, 두 Edge 필터, `MaxDistance`, 실제 노출. 노이즈까지 점으로 세지 않는다. |
| 두 대일 때 거리 이상 | 실제 선택 프로파일 → 방출 비교 → `Channel`, 간섭 검출 → 실제 modulation/auto-channel. |
| 정지 Depth 흔들림 | 진폭/간섭부터 확인 → Median/Gaussian → Temporal. 정적인 거리 편향은 시간 평균으로 해결되지 않는다. |
| 로봇 잔상 | `UseLastValue`, Temporal factor/threshold, Temporal HDR. 현재 프레임 비중을 늘려 반응과 흔들림을 비교. |
| Hz 부족 | Edge/Host/녹화 부하 → 화면/전송 모드 → 노출/HDR/DualBeam. `MaxDistance`나 주파수를 Hz 숫자처럼 조절하지 않는다. |

## 5. 최소 시험 순서

| 단계 | 바꾸는 것 | 채택 판단 |
|---|---|---|
| A. 기준 | 설정 유지, 단독/동시 방출만 비교 | 53을 유지한 채 52 방출 OFF/ON/OFF에서 53 Depth 변화를 기록. 이어서 52도 비교. 광학 간섭과 처리 부하를 구분. |
| B. 간섭 | 선택 파일/채널 확인 → 검출 0/400 비교 | 먼 기준점 거리 편향·무효 점·Hz 비교. 남으면 53 채널을 1→2→3으로 하나씩 시험하고 같은 기준 적용. 자동 채널/실제 적용 미확인은 기록. |
| C. 점 확보 | MinAmplitude만 변경 | 틀린 점을 제외한 유효 점 증가, 거리/흔들림 유지. 필터 OFF를 무조건 목표로 삼지 않는다. |
| D. 노이즈 | Median → Gaussian → Temporal을 각각 비교 | 정지 흔들림 감소 + 로봇 경계/잔상 허용. 이후 필요할 때 두 Edge 필터를 각각 비교. |
| E. Hz | 녹화 OFF/ON, Edge/Host, 화면 부하 비교 | Edge headless/RViz 부하부터 확인. Edge 자체가 느릴 때만 노출/HDR 실험. 전체 시야 요구가 있으면 ROI 축소로 Hz만 높이지 않는다. |
| F. 확정 | 선정 설정 + 모든 카메라 + 실제 녹화 + 재시작 | 세 요구조건과 거리 정확도를 전/후 비교하고 아래 기록 갱신. 실패 항목은 완료로 표시하지 않는다. |

단독에서도 거리 편향이 그대로 남으면 간섭만으로 설명하지 말고 반사/멀티패스, 노출·HDR, 센서 보정을 조사한다. 채널/필터 후보를 반복해도 편향이 남으면 제조사에 실제 modulation/auto-channel/펌웨어 상태를 확인한다. 하드웨어 동기화·교대 방출은 지원 여부를 확인한 후 선택하며, 현재 ROS에서 지원된다고 가정하지 않는다.

## 6. 확인·시험·녹화 명령

기존 fleet ROS 환경을 사용한다. 다음은 **162 예시**이며 161도 동일하게 실행한다. 기존 53/52 장비와 새 IP의 실제 시리얼 대응을 먼저 확인한다. 기존 드라이버가 떠 있으면 두 번째 드라이버를 띄우지 않는다.

```bash
# 각 Edge: 실제 선택 경로를 시작 로그에서 확인한 다음 백업한다.
source ~/SiriusRobotDT/setup/env.bash
cd "$SIRIUS_ROBOT_DT_ROOT"
NSL_TRIAL_DIR="/tmp/nsl_tuning_$(date +%Y%m%d_%H%M%S)"
mkdir -p "$NSL_TRIAL_DIR"
cp "$NSL_CALIB_DIR/N008B060D/params.yaml" "$NSL_TRIAL_DIR/params.before.yaml"
ros2 param dump /cam_162/roboscan_publish_node > "$NSL_TRIAL_DIR/ros.before.yaml"
ros2 param get /cam_162/roboscan_publish_node 'K. channel'
ros2 param get /cam_162/roboscan_publish_node 'C. imageType'

# 각 Edge / Host에서 따로 측정. timeout 종료 코드 124는 시간 제한 종료다.
timeout 30s ros2 topic hz /cam_162/camera/point_cloud --window 100
timeout 30s ros2 topic hz /cam_162/camera/point_cloud_rgb --window 100
ros2 topic info -v /cam_162/camera/point_cloud
```

위 `cp`는 로그가 해당 시리얼 파일을 선택한 경우의 예시다. 다른 경로가 선택됐다면 **그 파일**을 백업한다. `ros2 param dump` 결과는 진단용 ROS YAML이며 센서 `params.yaml`의 대체 파일이 아니다. 임시 백업은 작업 종료 전에 bag 옆 결과 폴더로 복사해 보관한다.

원시 XYZI cloud는 진단 구독자가 있을 때 추가 생성한다. `ros2 topic hz .../point_cloud`나 원본을 포함한 bag 녹화도 이 생성을 활성화하므로 일반 RGB cloud 운용과 원본 진단의 부하는 따로 기록한다.

아래는 작업자가 **B/D 단계에서 한 명령씩 적용·측정할 예시**이며, 일괄 실행할 프리셋이 아니다.

```bash
ros2 param set /cam_162/roboscan_publish_node 'V. interferenceDetectionLimit' 400
ros2 param set /cam_162/roboscan_publish_node 'T. temporalFilterFactor' 0.5
```

진단 bag은 Host에서 원시 Depth/XYZI를 우선 저장한다. RGB 및 추가 cloud를 전부 중복 저장하면 부하가 커지므로 짧게 비교한다.

```bash
mkdir -p ~/Desktop/ROSBAG
NSL_DIAG_BAG="$HOME/Desktop/ROSBAG/tuning_$(date +%Y-%m-%d_%H-%M-%S_%N)"
ros2 bag record -o "$NSL_DIAG_BAG" \
  /cam_161/camera/depth/image_raw /cam_161/camera/point_cloud \
  /cam_162/camera/depth/image_raw /cam_162/camera/point_cloud \
  /tf /tf_static
# Ctrl+C 종료 후:
ros2 bag info "$NSL_DIAG_BAG"
```

- 현재 `RGB_DISTANCE` 모드는 `/camera/ampl` 메시지를 내보내지 않는다. XYZI의 `intensity`에는 amplitude가 있다. 이미지가 필요할 때만 `C. imageType`을 `RGB_DISTANCE_AMPLITUDE`로 시험하고 `/cam_161/camera/ampl`, `/cam_162/camera/ampl`을 추가한다. 추가 모드의 Hz도 재측정한다.
- 기존 `mrb`는 RGB 압축 이미지·RGB cloud·두 로봇 pose·TF를 녹화한다. 최종 운용 비교에는 `mrb /cam_161/camera/depth/image_raw /cam_162/camera/depth/image_raw`로 Depth도 포함할 수 있다. 원하는 토픽이 실제 메시지를 내보내는지 확인한다.
- `ros2 topic hz`는 그 진단 구독자의 수신 속도다. Humble에서 `hz`에 QoS 옵션을 임의로 붙이지 않는다. bag의 토픽별 메시지 수/유효 기록 시간, 프레임 간격도 확인한다. 원시 cloud는 reliable이므로 수신기의 QoS도 점검한다.
- 한 bag에 같이 녹화해도 노출 동기화·광학 간섭 방지·무손실은 보장되지 않는다. Depth/cloud stamp는 SDK 프레임 수신 후 드라이버가 붙인 ROS 시각이며 카메라 간 하드웨어 노출 동기화 시각이 아니다.
- 재시험/복귀는 드라이버 종료 후 백업한 **센서 YAML**을 원래 경로에 복사하고 재시작해서 값·출력을 확인한다. 전원만 껐다 켜서는 자동 저장한 변경이 복구되지 않는다.

## 7. 작업 결과 — 튜닝할 때 반드시 갱신

아래 표는 새 튜닝 실행을 위한 기록 양식이다. 빈 항목은 현재 장치의 미검증 상태를 모두 집계한 진단 결과가 아니다.

작업일/담당: 미실행 · 최종 동시 카메라 수: 미측정 · 장면/기준거리: 미측정

정량 목표(있으면 기입): 유효 점/프레임 ______ · 흔들림 mm ______ · 녹화 Hz ______ · 거리 허용 오차 mm ______

| 시험/카메라 | 변경값·profile/serial | 단독/동시 | 유효 점/프레임 | 거리 오차 mm | 흔들림 P95−P5 mm | 녹화 Hz | 유효 점/초 | 잔상·판정 | bag/결과 경로 |
|---|---|---|---|---|---|---|---|---|---|
| 기준 / 52 | 실행 시 기입 | 동시 | 미측정 | 미측정 | 미측정 | 미측정 | 미측정 | 미측정 | 미기록 |
| 기준 / 53 | 실행 시 기입 | 동시 | 미측정 | 미측정 | 미측정 | 미측정 | 미측정 | 미측정 | 미기록 |
| 선정 / 52 | 실행 후 기입 | 동시 | 미측정 | 미측정 | 미측정 | 미측정 | 미측정 | 미측정 | 미기록 |
| 선정 / 53 | 실행 후 기입 | 동시 | 미측정 | 미측정 | 미측정 | 미측정 | 미측정 | 미측정 | 미기록 |

선정 이유/절충: 미작성 · 남은 문제: 미검증 · 재시작 후 확인: 미실행

### 다음 작업자에게 전달할 짧은 요청

```text
SiriusRobotDT/controller/src/NSL-3130AA-ROS2/tuning_guide.md를 기준으로 실제 카메라 튜닝을 진행해줘.
요구조건은 유효 pointcloud 많이, Depth fluctuation/노이즈 적게, 실제 동시 녹화 Hz 높게.
52/53 다중 운용 Depth 이상은 사용자 반복 관측으로 확인된 현상이다.
선택 프로파일 백업 → 방출 OFF/ON/OFF → 한 항목씩 비교 → 모든 카메라 녹화/재시작 검증 순서로 진행해.
기존 설정을 최적값이라고 가정하지 말고 YAML/ROS 값 변환, HDR 순서, 자동 저장에 주의해.
완료 후 이 문서의 진행 상태·실측 결과표·선정 설정 경로를 갱신해.
실측한 항목만 확인됨으로 바꾸고, 측정 못 한 항목과 절충은 명시해.
```

## 근거와 코드 진입점

- [프로파일 선택/launch](roboscan_nsl3130/launch/camera.launch.py): `_resolve_params_file`, 토픽 remap.
- [드라이버](roboscan_nsl3130/src/roboscan_publish_node.cpp): `parametersCallback`, `setReconfigure`, `publishFrame`, `publishCalibratedRgbCloud`.
- [YAML 읽기/저장](roboscan_nsl3130/include/roboscan_publish_node.hpp): `load_sensor_tuning_params`, `save_params`.
- [동봉 SDK 매뉴얼](roboscan_nsl3130/nsl_lib/DOC/ROBOSCAN%28NSL-3140AA%29%20SDK%20Software%20Manual.pdf), ver 1.12: p.11 노출/진폭, p.13 채널, p.14 필터, p.19 자동 채널. 장비별 동작은 해당 펌웨어에서 확인한다.
- [ToF 외부 광원·다중 반사 원리 — Basler 공식 문서](https://docs.baslerweb.com/influencing-factors). 타사 동기화 기능/수치가 NSL에도 지원된다는 뜻은 아니다.
