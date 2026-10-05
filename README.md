# NSL 카메라 모듈

`roboscan_nsl3130`은 NSL-3130AA RGB-D 드라이버·시리얼 감지·RGB 압축·intrinsic/extrinsic/STag 보정을 제공한다. `rqt_reconfigure_combo`는 센서 파라미터 편집 UI다. 기존 NSL-3130AA-ROS2 소스를 SiriusRobotDT 안으로 통합했다.

```bash
source ~/SiriusRobotDT/setup/env.bash
cd ~/SiriusRobotDT/controller
colcon build --symlink-install --packages-select roboscan_nsl3130 rqt_reconfigure_combo
source install/setup.bash
# Edge에서 카메라 시작
ros2 launch roboscan_nsl3130 camera.launch.py namespace:=cam_161
# Host에서 확인
mvw solver:=false solver_writeback:=false
```

시리얼별 실제 설정과 보정은 `config/calibration/<serial>/`에 저장한다. `NSL_CALIB_DIR`로 저장 경로를 바꿀 수 있다. 공통 센서 설정은 패키지의 `lidar_params.yaml`, 보정 실행용 설정은 `lidar_params_calibration.yaml`이며 선택 규칙과 각 파라미터의 영향은 [튜닝 가이드](../../../docs/camera/tuning_guide.md)에 정리했다.

[실행 명령·Edge 운영](../../../exec/README.md) · [프로젝트 README](../../../README.md)

전체 설정은 `~/SiriusRobotDT/config/config.yaml`, 실제 SDK 파일은 `config/calibration/<serial>/`에 있다. Edge LAN `192.168.3.161..165`가 namespace를 결정하며 센서 링크 IP는 전역 config에서 따로 지정한다. USB 시리얼을 감지하고 검증된 writeback을 저장하면 전역 config에도 실제 바인딩과 보정 파일 해시를 기록한다.
