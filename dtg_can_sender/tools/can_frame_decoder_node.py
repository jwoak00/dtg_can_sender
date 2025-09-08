#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from can_msgs.msg import Frame
from std_msgs.msg import Float32, Int32, Bool
import math


def u16_le(b0, b1): return (b0 | (b1 << 8)) & 0xFFFF

def s16_le(b0, b1): return int.from_bytes(bytes([b0, b1]), 'little', signed=True)


class CanFrameDecoder(Node):
    def __init__(self):
        super().__init__('can_frame_decoder')
        can_topic = self.declare_parameter('can_topic', '/transmit/frame').get_parameter_value().string_value
        self.altitude_scale = self.declare_parameter('altitude_scale', 1.0).get_parameter_value().double_value
        self.sub = self.create_subscription(Frame, can_topic, self.on_frame, 100)

        ns = '/debug/can'
        # 0x100
        self.pub_100_speed = self.create_publisher(Float32, f'{ns}/id_0x100/speed_kmh', 10)
        self.pub_100_rpm = self.create_publisher(Float32, f'{ns}/id_0x100/rpm', 10)
        self.pub_100_steer = self.create_publisher(Float32, f'{ns}/id_0x100/steering_deg', 10)
        self.pub_100_gear = self.create_publisher(Int32, f'{ns}/id_0x100/gear_raw', 10)
        self.pub_100_speed_mps = self.create_publisher(Float32, f'{ns}/id_0x100/speed_mps', 10)
        self.pub_100_steer_rad = self.create_publisher(Float32, f'{ns}/id_0x100/steering_rad', 10)

        # 0x200
        self.pub_200_auto = self.create_publisher(Bool, f'{ns}/id_0x200/auto_on', 10)
        self.pub_200_steer = self.create_publisher(Float32, f'{ns}/id_0x200/steering_deg', 10)
        self.pub_200_speed = self.create_publisher(Float32, f'{ns}/id_0x200/speed_kmh', 10)
        self.pub_200_accel = self.create_publisher(Float32, f'{ns}/id_0x200/accel_ms2', 10)
        self.pub_200_speed_mps = self.create_publisher(Float32, f'{ns}/id_0x200/speed_mps', 10)
        self.pub_200_steer_rad = self.create_publisher(Float32, f'{ns}/id_0x200/steering_rad', 10)

        # 0x400 (Altitude는 항상 0)
        self.pub_400_alt = self.create_publisher(Float32, f'{ns}/id_0x400/altitude', 10)
        self.pub_400_yaw = self.create_publisher(Float32, f'{ns}/id_0x400/yaw_deg', 10)
        self.pub_400_ax = self.create_publisher(Float32, f'{ns}/id_0x400/ax_ms2', 10)
        self.pub_400_ay = self.create_publisher(Float32, f'{ns}/id_0x400/ay_ms2', 10)

        # 0x201 (accel/brake normalized)
        self.pub_201_accel_norm = self.create_publisher(Float32, f'{ns}/id_0x201/accel_norm', 10)
        self.pub_201_brake_norm = self.create_publisher(Float32, f'{ns}/id_0x201/brake_norm', 10)
        self.pub_201_accel_est = self.create_publisher(Float32, f'{ns}/id_0x201/accel_est', 10)   # 0~127 추정
        self.pub_201_brake_est = self.create_publisher(Float32, f'{ns}/id_0x201/brake_est', 10)   # 0~4000 추정

    def on_frame(self, msg: Frame):
        d = msg.data
        fid = msg.id & 0x7FF
        if fid == 0x200:
            auto_on_raw = (int(d[0]) & 0xF0) >> 4
            auto_on = bool(auto_on_raw)
            steering_deg = u16_le(d[1], d[2]) * 0.0625 - 2048.0
            speed_kmh = u16_le(d[3], d[4]) * (1.0 / 256.0)
            accel_ms2 = u16_le(d[5], d[6]) * 0.01 - 5.0
            self.pub_200_auto.publish(Bool(data=auto_on))
            self.pub_200_steer.publish(Float32(data=float(steering_deg)))
            self.pub_200_speed.publish(Float32(data=float(speed_kmh)))
            self.pub_200_accel.publish(Float32(data=float(accel_ms2)))
            self.pub_200_steer_rad.publish(Float32(data=float(steering_deg * math.pi / 180.0)))
            self.pub_200_speed_mps.publish(Float32(data=float(speed_kmh / 3.6)))

        elif fid == 0x100:
            speed_kmh = u16_le(d[0], d[1]) * (1.0 / 256.0)
            rpm = u16_le(d[2], d[3]) * 0.125
            steering_deg = u16_le(d[4], d[5]) * 0.0625 - 2048.0
            self.pub_100_speed.publish(Float32(data=float(speed_kmh)))
            self.pub_100_rpm.publish(Float32(data=float(rpm)))
            self.pub_100_steer.publish(Float32(data=float(steering_deg)))
            self.pub_100_gear.publish(Int32(data=int(d[6])))
            self.pub_100_steer_rad.publish(Float32(data=float(steering_deg * math.pi / 180.0)))
            self.pub_100_speed_mps.publish(Float32(data=float(speed_kmh / 3.6)))

        elif fid == 0x400:
            if msg.dlc < 8:
                self.get_logger().warn('0x400 frame dlc < 8 (got %d), skip', msg.dlc)
                return
            altitude = 0.0  # 고정
            yaw_deg = u16_le(d[2], d[3]) * 0.01
            ax = s16_le(d[4], d[5]) * 0.01
            ay = s16_le(d[6], d[7]) * 0.01
            self.pub_400_alt.publish(Float32(data=float(altitude)))
            self.pub_400_yaw.publish(Float32(data=float(yaw_deg)))
            self.pub_400_ax.publish(Float32(data=float(ax)))
            self.pub_400_ay.publish(Float32(data=float(ay)))

        elif fid == 0x201:
            # accel_status(0~127), brake_status(0~4000) 를 0~255 정규화 했다고 가정한 프레임
            # data[0]=정규화 accel(0~255), data[1]=정규화 brake(0~255)
            accel_norm = int(d[0])
            brake_norm = int(d[1])
            # 추정 원래값 역계산 (옵션): accel_est(0~127), brake_est(0~4000)
            accel_est = accel_norm * (127.0 / 255.0)
            brake_est = brake_norm * (4000.0 / 255.0)
            self.pub_201_accel_norm.publish(Float32(data=float(accel_norm)))
            self.pub_201_brake_norm.publish(Float32(data=float(brake_norm)))
            self.pub_201_accel_est.publish(Float32(data=float(accel_est)))
            self.pub_201_brake_est.publish(Float32(data=float(brake_est)))


def main():
    rclpy.init()
    node = CanFrameDecoder()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()