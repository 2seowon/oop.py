import math

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, QoSReliabilityPolicy, QoSHistoryPolicy, QoSDurabilityPolicy

from px4_msgs.msg import OffboardControlMode
from px4_msgs.msg import TrajectorySetpoint
from px4_msgs.msg import VehicleStatus
from px4_msgs.msg import VehicleOdometry
from px4_msgs.msg import VehicleCommand
from px4_msgs.msg import VehicleControlMode # 메시지 추가: VehicleControlMode 메시지를 구독하여 오프보드 모드 활성화 여부를 확인할 수 있도록 합니다.

class FlightTest(Node):
    def __init__(self):
        super().__init__('real_flight_test') # 노드 이름 : real_flight _test 

        self.offboard_enabled = False
        self.armed = VehicleStatus.ARMING_STATE_DISARMED

        # [추가] 처음 close_enough 계산 시 self.position이 없어 발생하는 에러 방지
        self.position = [0.0, 0.0, 0.0]

        # waypoints for the drone to target (실수를 방지하기 위해 float 형태로 명시하는 것이 좋습니다)
        self.target = [
            [0.0, 0.0, -10.0],
            [5.0, 0.0, -10.0],
            [5.0, 5.0, -10.0],
            [5.0, -5.0, -10.0],
            [5.0, 0.0, -10.0],
            [0.0, 0.0, -10.0]
        ]

        self.target_index = 0
        self.reached_target_counter = 0
        self.distance_threshold = 2.0
        self.hold_threshold = 30 # 내 코드에는 hold -> time으로 변경 + 3초 머무르도록 300 으로 바꿈 

        qos_profile_pub = QoSProfile(
            reliability=QoSReliabilityPolicy.BEST_EFFORT,
            durability=QoSDurabilityPolicy.TRANSIENT_LOCAL,
            history=QoSHistoryPolicy.KEEP_LAST,
            depth=0
        )

        qos_profile_sub = QoSProfile(
            reliability=QoSReliabilityPolicy.BEST_EFFORT,
            durability=QoSDurabilityPolicy.VOLATILE,
            history=QoSHistoryPolicy.KEEP_LAST,
            depth=0
        )

        self.subscriber_odometry = self.create_subscription(VehicleOdometry, '/fmu/out/vehicle_odometry', self.callback_odometry, qos_profile_sub)
        self.subscriber_vehicle_status = self.create_subscription(VehicleStatus, '/fmu/out/vehicle_status_v1', self.callback_vehicle_status, qos_profile_sub)
        self.subscriber_vehicle_control_mode = self.create_subscription(VehicleControlMode, '/fmu/out/vehicle_control_mode', self.callback_vehicle_control_mode, qos_profile_sub)

        self.publisher_offboard_mode = self.create_publisher(OffboardControlMode, '/fmu/in/offboard_control_mode', qos_profile_pub)
        self.publisher_trajectory_setpoint = self.create_publisher(TrajectorySetpoint, '/fmu/in/trajectory_setpoint', qos_profile_pub)
        self.publisher_vehicle_command = self.create_publisher(VehicleCommand, '/fmu/in/vehicle_command', qos_profile_pub)

        timer_period = 0.01
        self.timer = self.create_timer(timer_period, self.callback_cmdloop)

    def callback_odometry(self, msg):
        self.position = msg.position

    def callback_vehicle_status(self, msg):
        self.armed = msg.arming_state

    def callback_vehicle_control_mode(self, msg):
        self.offboard_enabled = msg.flag_control_offboard_enabled

    def publish_offboard_mode(self):
        msg = OffboardControlMode()
        msg.timestamp = int(self.get_clock().now().nanoseconds / 1000)
        msg.position = True
        msg.velocity = False
        msg.acceleration = False
        msg.attitude = False
        msg.body_rate = False
        msg.thrust_and_torque = False
        msg.direct_actuator = False
        self.publisher_offboard_mode.publish(msg)

    def publish_vehicle_command(self, command, param1, param2):
        msg = VehicleCommand()
        msg.timestamp = int(self.get_clock().now().nanoseconds / 1000)
        msg.param1 = float(param1)
        msg.param2 = float(param2)
        msg.command = command
        msg.from_external = True
        self.publisher_vehicle_command.publish(msg)

    # 드론에 내릴 명령 설정 (def publish_vehicle_command() 메소드 이용) 
    def publish_arm_command(self):
        self.publish_vehicle_command(VehicleCommand.VEHICLE_CMD_COMPONENT_ARM_DISARM, 1.0, 0.0)
        print("Arm command send")   

    def publish_disarm_command(self):
        self.publish_vehicle_command(VehicleCommand.VEHICLE_CMD_COMPONENT_ARM_DISARM, 0.0, 0.0)
        print("Disarm command send")

    def publish_trajectory_setpoint(self, p):
        msg = TrajectorySetpoint()
        msg.timestamp = int(self.get_clock().now().nanoseconds / 1000)
        msg.position = p
        self.publisher_trajectory_setpoint.publish(msg)

    def close_enough(self):
        # [수정 2] 정의되지 않았던 sc 변수 대신 self.target_index를 직접 사용합니다.
        distance_to_target = math.dist(self.position, self.target[self.target_index])
        if distance_to_target < self.distance_threshold:
            return True
        else:
            return False

    def callback_cmdloop(self):
        if self.armed == VehicleStatus.ARMING_STATE_DISARMED:
            self.publish_arm_command()
            self.publish_vehicle_command(VehicleCommand.VEHICLE_CMD_DO_SET_MODE, 1.0, 6.0)

        self.publish_offboard_mode()

        # [수정 1] 오타 수정: target 리스트에서 인덱스로 좌표를 제대로 가져오도록 수정했습니다.
        self.publish_trajectory_setpoint(self.target[self.target_index])

        if self.close_enough():
            self.reached_target_counter += 1
            if self.reached_target_counter > self.hold_threshold:
                # [수정 4] 마지막 타겟에 도착한 뒤 인덱스가 초과하여 코드가 멈추는(IndexError) 현상을 방지합니다.
                if self.target_index < len(self.target) - 1:
                    self.target_index += 1
                    self.reached_target_counter = 0
                else:
                    # 마지막 웨이포인트(0, 0, -10)에 도달하면 인덱스를 올리지 않고 그 자리에서 호버링합니다.
                    pass

def main(args=None):
    rclpy.init(args=args)
    real_flight_test = FlightTest()
    rclpy.spin(real_flight_test)
    real_flight_test.destroy_node()
    rclpy.shutdown()

# [수정 3] 파이썬 표준에 맞게 언더바를 2개씩 붙여 코드가 정상적으로 실행되도록 수정했습니다.
if __name__ == '__main__':
    main()