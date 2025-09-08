# DTG CAN Sender/Receiver for Autoware

## 1. 개요

`dtg_can_sender`는 [Autoware](https://autoware.ai/) 환경에서 차량 제어 및 상태 정보를 CAN (Controller Area Network) 버스로 송신하고, CAN 버스로부터 수신된 데이터를 ROS2 토픽으로 변환하여 모니터링하는 ROS2 패키지입니다.

이 패키지는 C++로 작성된 **Sender 노드**와 Python으로 작성된 **Decoder 노드**로 구성되어 있으며, 특정 CAN 프로토콜(`DTG_CAN_Protocol.xlsx` 기반)에 맞춰 설계되었습니다.

## 2. 주요 기능

* [cite_start]**Autoware 토픽 → CAN 메시지 변환 (인코딩)**: Autoware의 다양한 제어 및 상태 토픽을 구독하여 CAN 프레임으로 인코딩 후 발행합니다[cite: 1].
* [cite_start]**CAN 메시지 → ROS2 토픽 변환 (디코딩)**: CAN 프레임을 수신하여 각 데이터 필드를 물리 단위로 디코딩한 후, 디버깅 및 모니터링을 위한 ROS2 토픽으로 발행합니다[cite: 2].
* [cite_start]**실시간 차량 상태 반영**: 차량의 실제 속도, 조향각, 기어, RPM 등의 상태 정보를 CAN 메시지(`0x100`)에 담아 전송합니다[cite: 1].
* [cite_start]**자율주행 제어 명령 전송**: Autoware로부터 받은 목표 속도, 목표 조향각, 가속도 등의 제어 명령을 CAN 메시지(`0x200`)로 전송합니다[cite: 1].
* [cite_start]**위치 및 IMU 데이터 전송**: Odometry 정보를 기반으로 계산된 Yaw, 종방향/횡방향 가속도를 CAN 메시지(`0x400`)에 담아 전송합니다[cite: 1].

## 3. 노드 상세

### 3.1. `dtg_can_sender` (C++)

* **역할**: Autoware의 주요 토픽을 CAN 메시지로 변환(인코딩)하여 발행하는 핵심 노드입니다.
* [cite_start]**실행 파일**: `dtg_can_sender::DtgCanSender` [cite: 1]
* **구독 (Subscribed) 토픽**:
    * [cite_start]`/control/command/control_cmd` (`autoware_control_msgs::msg::Control`): 목표 속도, 조향각, 가속도 등 제어 명령 수신 [cite: 1]
    * [cite_start]`/system/operation_mode/state` (`autoware_adapi_v1_msgs::msg::OperationModeState`): 자율주행 모드 상태 수신 [cite: 1]
    * [cite_start]`/vehicle/status/velocity_status` (`autoware_vehicle_msgs::msg::VelocityReport`): 현재 차량 속도 수신 [cite: 1]
    * [cite_start]`/vehicle/status/steering_status` (`autoware_vehicle_msgs::msg::SteeringReport`): 현재 조향각 수신 [cite: 1]
    * [cite_start]`/vehicle/status/gear_status` (`autoware_vehicle_msgs::msg::GearReport`): 현재 기어 상태 수신 [cite: 1]
    * [cite_start]`/vehicle/status/engine_rpm` (`std_msgs::msg::Float32`): 현재 엔진 RPM 수신 [cite: 1]
    * [cite_start]`/localization/kinematic_state` (`nav_msgs::msg::Odometry`): 차량 위치 및 자세, 속도 정보 수신 (Yaw, 가속도 계산용) [cite: 1]
    * [cite_start]`/vehicle/status/actuation_status` (`tier4_vehicle_msgs::msg::ActuationStatusStamped`): 액추에이터 상태(악셀/브레이크 페달) 수신 [cite: 1]
* **발행 (Published) 토픽**:
    * [cite_start]`/transmit/frame` (`can_msgs::msg::Frame`): 인코딩된 CAN 메시지 발행 (기본값, 파라미터로 변경 가능) [cite: 1]

### 3.2. `can_frame_decoder_node.py` (Python)

* **역할**: `dtg_can_sender` 또는 실제 CAN 드라이버로부터 받은 CAN 메시지를 사람이 보기 쉬운 ROS2 토픽으로 디코딩하여 발행하는 디버깅용 유틸리티 노드입니다.
* [cite_start]**실행 파일**: `can_frame_decoder_node.py` [cite: 2]
* **구독 (Subscribed) 토픽**:
    * [cite_start]`/transmit/frame` (`can_msgs::msg::Frame`): 디코딩할 CAN 메시지 수신 (기본값, 파라미터로 변경 가능) [cite: 2]
* [cite_start]**발행 (Published) 토픽**[cite: 2]:
    * `/debug/can/id_0x100/*`: 속도(km/h, m/s), RPM, 조향각(deg, rad) 등
    * `/debug/can/id_0x200/*`: 자율주행 상태, 목표 속도, 목표 조향각 등
    * `/debug/can/id_0x201/*`: 정규화된 악셀/브레이크 값 및 추정 원본 값
    * `/debug/can/id_0x400/*`: Yaw, 종방향/횡방향 가속도(ax, ay), 고도

## 4. CAN 프로토콜

이 패키지는 다음의 CAN ID와 데이터 레이아웃을 사용합니다.

| CAN ID | 설명 | 주요 데이터 필드 (Little Endian) |
| :--- | :--- | :--- |
| **0x100** | **차량 현재 상태** | `Speed` (속도), `RPM`, `Steering` (조향각), `Gear Pos` (기어 위치) |
| **0x200** | **자율주행 제어 명령** | `Autonomous Mode`, `Target Steering`, `Target Speed`, `Target Accel` |
| **0x201** | **액추에이터 상태** | `Accel Pedal` (악셀 페달 값), `Brake Pedal` (브레이크 페달 값) |
| **0x400** | **IMU 및 위치 정보**| `Altitude` (고도-현재는 0), `Yaw`, `Accel_x` (종방향 가속도), `Accel_y` (횡방향 가속도) |

*자세한 스케일 및 오프셋 값은 소스 코드(`dtg_can_sender.cpp`, `can_frame_decoder_node.py`)를 참고하십시오.*

## 5. 빌드 방법

1.  이 패키지를 colcon 워크스페이스의 `src` 폴더 안에 위치시킵니다.
2.  워크스페이스 루트에서 다음 명령어를 실행하여 빌드합니다.
    ```bash
    colcon build --symlink-install --packages-select dtg_can_sender
    ```

## 6. 사용 방법

### 6.1. Sender 노드 실행

다음 launch 파일을 사용하여 `dtg_can_sender` 노드를 실행할 수 있습니다.

```bash
ros2 launch dtg_can_sender dtg_can_sender.launch.xml
```

#### 파라미터

launch 파일에서 다음 파라미터를 수정하여 노드 동작을 변경할 수 있습니다.

* `can_tx_topic`: 발행할 CAN 메시지 토픽 이름 (기본값: `/transmit/frame`)
* `operation_mode_topic`: 구독할 자율주행 모드 토픽 이름 (기본값: `/system/operation_mode/state`)
* `timer_period_ms`: CAN 메시지 발행 주기 (밀리초, 기본값: 100)

### 6.2. Decoder 노드 실행 (디버깅용)

Sender가 발행하는 CAN 메시지를 디코딩하여 확인하려면 별도의 터미널에서 다음 명령어를 실행합니다.

```bash
ros2 run dtg_can_sender can_frame_decoder_node.py
