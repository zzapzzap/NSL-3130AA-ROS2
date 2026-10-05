# NSL 카메라 모듈

`roboscan_nsl3130`은 NSL-3130AA RGB-D 드라이버·시리얼 감지·RGB 압축·intrinsic/extrinsic/STag 보정을 제공한다. `rqt_reconfigure_combo`는 센서 파라미터 편집 UI다. 기존 NSL-3130AA-ROS2 소스를 SiriusRobotDT 안으로 통합했다.

```bash
source ~/SiriusRobotDT/setup/env.bash
cd ~/SiriusRobotDT/controller
colcon build --symlink-install --packages-select roboscan_nsl3130 rqt_reconfigure_combo
source install/local_setup.bash
# Edge에서 카메라 시작
ros2 launch roboscan_nsl3130 camera.launch.py namespace:=cam_161
# Host에서 확인
mvw
```

통합 실행은 공통 [config/config.yaml](../../../config/config.yaml)의 `cameras.cam_<옥텟>`에 기록된 Edge IP·serial·보정 폴더·intrinsic·extrinsic·전역 TF를 사용한다. ROOT의 `parameters`는 그 `config.yaml` 디렉터리 기준 상대 경로다. 모듈 단독 실행은 [setup/fleet.yaml](setup/fleet.yaml)을 사용하며, 이 파일의 `parameters`는 `setup/` 기준 상대 경로다. 원래 SDK 설정·intrinsic·extrinsic·STag 결과는 `calib_output/<실제 시리얼>/`에 유지한다. Edge LAN `192.168.3.161..165`가 `cam_161..cam_165`를 결정하며, 센서 전용 NIC 주소는 native fleet의 `sensor_host`, `sensor_source_address`를 따른다. 시리얼은 Edge가 실제 장치를 확인한 뒤 자동 기록하며 LAN 주소로 추측하지 않는다.

`/<cam_N>/camera/device_info`는 실제 USB 시리얼과 보정 파일의 SHA256을 반환한다. `/<cam_N>/camera/profile/get`은 기존 GetWeight 서비스로 보정 원본을 전송한다. Ethernet SDK에는 제조사 시리얼 조회 함수가 없어 USB 정보를 얻지 못하면 `verified=false`와 캐시 정보를 반환한다. USB 카메라가 바뀌면 새 실제 시리얼의 SDK 폴더를 사용한다. 일부 파일만 있는 사용자 프로필에서는 누락된 intrinsic/extrinsic을 같은 실제 시리얼 폴더에서 읽는다.

[calib_output/global_frames.yml](calib_output/global_frames.yml)은 map 기준 마커·카메라 TF와 마커별 카메라 TF를 함께 보관한다. 저장 bag에서 `stag_marker`는 marker 0 기준이며, marker 7 위치는 기록된 `stag_marker → tag_7`로 계산했다. `reference`는 그 실제 기준과 원래 수동 map 배치를 명시한다. 편집할 마커를 바꿔도 SDK의 `stag_marker` 기준을 임의로 바꾸지 않는다. 마커별 상대 TF는 `inverse(T_map_marker) × T_map_camera`로 계산하며 원본 SDK 결과를 덮어쓰지 않는다.

이 기준은 해당 bag의 근거다. 새 live 카메라의 SDK `marker_id`가 다르면 동일한 map 기준이라고 가정하지 않는다. [기준 관측](calib_output/runtime/reference_observations.yml)과 실제 시리얼의 `multiview.yml`을 비교한다. `runtime/`에는 녹화된 RGB–LiDAR TF 관측과 보정 파일 해시 목록도 보관한다.

공통 센서 설정은 `roboscan_nsl3130/lidar_params.yaml`, 보정용 설정은 `lidar_params_calibration.yaml`이다. 파라미터 역할·간섭 분리·점 수/노이즈/Hz 평가 순서는 [튜닝 가이드](tuning_guide.md)를 따른다. 센서·보정·Edge 서비스 설치 스크립트는 이 모듈의 `setup/`에 있다. [프로젝트 setup](../../../README.md)에서 역할을 확인한 뒤 실행한다.

```bash
# 네트워크/서비스를 변경하지 않고 역할 확인
bash ~/SiriusRobotDT/setup/setup.bash --check
# 명시적으로 원격 YAML을 가져오기 (환경 source만으로 실행되지 않음)
mcalpull 161 /home/edge/SiriusRobotDT/controller/src/NSL-3130AA-ROS2/calib_output/N_ACTUAL_SERIAL
```

`mcalpull`은 현재 지정한 Edge IP와 `parameters` 폴더에 YAML만 설치한다. 경로를 가져온 것만으로 시리얼을 검증했다고 기록하지 않는다. [프로젝트 README](../../../README.md)

Live와 bag은 ROS domain 42를 공유한다. `sensors start|stop|status`로 Host main unit을 제어하며, main 정지 시 Edge camera unit도 `ExecStopPost`로 정지한다. `--start-edge 161 --pose false`는 camera 전용, `--pose true`는 camera와 단일뷰 inference를 함께 실행한다. `mloc`은 수동 localization 편집기를 다시 연다.

Host와 Edge의 `timedatectl show -p NTPSynchronized --value` 및 `date -u --iso-8601=ns`를 비교해 시간 동기화를 확인한다. 각 OS의 기존 NTP client를 같은 시간 원본에 연결한다. Host `.166`을 NTP 원본으로 지정할 때는 그 Host의 NTP 서비스도 동작해야 한다. `msb`는 카메라 서비스를 재시작하며 시계를 변경하지 않는다. 시간 차이나 이전 ready 메시지는 `mtr`에서 새 모델 준비 완료로 인정하지 않는다.
