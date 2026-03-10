import math 
import sys 

import rclpy 
from rclpy.node import Node
from rclpy.qos import QoSProfile, QoSReliabilityPolicy, QoSHistoryPolicy, QoSDurabilityPolicy

from px4_msgs.msg import OffboardControlMode # 퍼블리시 : 드론에게 오프보드 모드 명령
from px4_msgs.msg import TrajectorySetpoint # 퍼블리시 : 드론에게 목표 위치와 자세 정보를 보냄
from px4_msgs.msg import VehicleStatus # 구독 : 드론의 상태 정보
from px4_msgs.msg import VehicleOdometry # 구독 : 드론의 위치와 자세 정보 
from px4_msgs.msg import VehicleCommand # 퍼블리시 : 드론에게 명령을 보냄 
from px4_msgs.msg import VehicleControlMode # 구독 : 드론의 현재 제어 모드 

class FlightTest(Node):
    def __init__(self):
        super().__init__('flight_test') # 노드 이름 : flight_test 


        # 기본 변수 설정 
        self.offboard_enabled = False # offboard_enabeld 라는 변수를 False로 초기화 
        self.armed = VehicleStatus.ARMING_STATE_DISARMED # 처음에는 disarm으로 시작 
        
        self.position = [0.0, 0.0, 0.0]
        # 드론이 이동할 target 지점 설정 
        self.target = [
            [0.0, 0.0, -10.0],
            [5.0, 0.0, -10.0],
            [5.0, 5.0, -10.0],
            [5.0, -5.0, -10.0],
            [5.0, 0.0, -10.0],
            [0.0, 0.0, -10.0]
        ]

        self.target_index = 0 # waypoiny index 초기화 
        self.reached_target_counter = 0 # 목표 반경에 머문 시간 카운터 초기화 
        self.distance_threshold = 2.0 # 목표 반경 2.0m로 설정 
        self.time_threshold = 300 # 목표 반경에 머무르는 시간 3초 설정 (30->300으로 변경) 

        # 발행용 QoS
        qos_profile_pub = QoSProfile(
            reliability=QoSReliabilityPolicy.BEST_EFFORT,
            durability=QoSDurabilityPolicy.TRANSIENT_LOCAL, # 구독 시점 이전 메시지도 받을 수 있도록 설정 ( publisher )
            history=QoSHistoryPolicy.KEEP_LAST, 
            depth=0
        )

        # 구독용 QoS
        qos_profile_sub = QoSProfile(
            reliability=QoSReliabilityPolicy.BEST_EFFORT,
            durability=QoSDurabilityPolicy.VOLATILE, # 구독 시점 이후 메시지만 받도록 설정 ( subscriber )
            history=QoSHistoryPolicy.KEEP_LAST,
            depth=0
        )

        # subsriber (callback 함수 필요)
        self.subscriber_odometry = self.create_subscription(
            VehicleOdometry, 
            '/fmu/out/vehicle_odometry', 
            self.callback_odometry,
            qos_profile_sub 
        )
        # 드론 자세 정보 구독 : /fmu/out/vehicle_odometry 토픽을 구독하여 드론의 위치와 자세 정보를 받아오는 콜백 함수 설정

        self.subscriber_vehicle_status = self.create_subscription(
            VehicleStatus,
            '/fmu/out/vehicle_status_v1',
            self.callback_vehicle_status,
            qos_profile_sub
        )
        # 드론의 현재 상태를 구독 

        self.subscriber_vehicle_control_mode = self.create_subscription(
            VehicleControlMode,
            '/fmu/out/vehicle_control_mode',
            self.callback_vehicle_control_mode, 
            qos_profile_sub
        )
        # 드론의 현재 제어 모드 정보를 구독 

        
        # Publisher 
        self.publisher_offboard_mode = self.create_publisher(
            OffboardControlMode,
            '/fmu/in/offboard_control_mode',
            qos_profile_pub
        )
        # 오프보드 명령을 발행 

        self.publisher_trajectory_setpoint = self.create_publisher(
            TrajectorySetpoint,
            '/fmu/in/trajectory_setpoint',
            qos_profile_pub
        )
        # 타겟 정보를 발행 

        self.publisher_vehicle_command = self.create_publisher(
            VehicleCommand,
            '/fmu/in/vehicle_command',
            qos_profile_pub
        )
        # 드론에게 내리는 모든 명령을 [publisehr_vehicle_command 에서 사용 ]


        # 0.01초 주기로 타이머 생성 (self.callback_cmdloop)
        timer_period = 0.01 
        self.timer = self.create_timer(timer_period , self.callback_cmdloop)
        # 메인 함수 실행 (callback_cmdloop)
                                                                
    # 구독 토픽에서 실행할 call back 함수 설정 
    #1. subscriber_odometry에서 실행 : 드론의 현재 위치를 객체에 넣음 
    def callback_odometry(self,msg):
        self.position = msg.position  
    
    #2. subscriber_vehicle_status에서 실행 : 드론의 arm 상태를 가져옴 
    def callback_vehicle_status(self,msg):
        self.armed = msg.arming_state  

    #3. subscriber_vehicle_control_mode 에서 실행 : 드론의 오프보드 제어 모드 실행 여부를 가져옴 
    def callback_vehicle_control_mode(self,msg):
        self.offboard_enabled = msg.flag_control_offboard_enabled 

    # 발행자에서 발행할 데이터를 정의하는 함수들 
    #1. publisher_offboard_mode에서 발행할 데이터 함수 
    def publish_offboard_mode(self):
        msg = OffboardControlMode()
        msg.timestamp = int(self.get_clock().now().nanoseconds / 1000) # 데이터 발행 시각 (마이크로초)
        msg.position = True  #위치 기반 오프보드 제어 
        msg.velocity = False 
        msg.acceleration = False 
        msg.attitude = False 
        msg.body_rate = False 
        msg.thrust_and_torque = False 
        msg.direct_actuator = False 
        self.publisher_offboard_mode.publish(msg) #발행자가 위의 메시지들을 발행함 

    #2. publisher_vehicle_command에서 실행할 명령 관련 매개변수 설정 
    # 명령을 내릴 때 사용할 메소드를 정의한다고 생각 
    def publish_vehicle_command(self,command, param1, param2):
        msg = VehicleCommand()
        msg.timestamp = int(self.get_clock().now().nanoseconds/1000) #발행 메시지 시각 
        msg.param1 = float(param1) # 리모컨 처럼 코드로 변수를 가져와서 명령할 수 있게 설정
        msg.param2 = float(param2)
        msg.command = command 
        msg.from_external = True # 외부 컴퓨터 명령 true로 설정     
        self.publisher_vehicle_command.publish(msg)

    # def publish_vehicle_command() 메소드를 이용해서 드론에 내릴 명령들 설정 

    #2-1. arm 명령 (param1 = 1.0)
    def publish_arm_command(self):
        self.publish_vehicle_command(VehicleCommand.VEHICLE_CMD_COMPONENT_ARM_DISARM, 1.0, 0.0)
        print("Arm command send") 
    
    #2-2. disarm 명령 (param = 0.0)
    def publish_disarm_command(self):
        self.publish_vehicle_command(VehicleCommand.VEHICLE_CMD_COMPONENT_ARM_DISARM, 0.0, 0.0)
        print("Disarm command send") 
    
    #3. publisher_trajectory_setpoint 에서 내릴 목표 위치  
    def publish_trajectory_setpoint(self,p):
        msg = TrajectorySetpoint()
        msg.timestamp = int(self.get_clock().now().nanoseconds/1000) #발행 메시지 시각 
        msg.position = p 
        self.publisher_trajectory_setpoint.publish(msg)
    
    def close_enough(self):
        distance_to_target = math.dist(self.position , self.target[self.target_index]) #math.dist로 직선 거리 계산 
        if distance_to_target < self.distance_threshold:
            return True              # 기준 거리보다 직선 거리가 작아지면 

        else:
            return False
        
    def callback_cmdloop(self):
        if self.armed == VehicleStatus.ARMING_STATE_DISARMED:
            self.publish_arm_command() # disarm 이면 arm으로 바꾸는 메소드 실행 
            self.publish_vehicle_command(VehicleCommand.VEHICLE_CMD_DO_SET_MODE,1.0,6.0)  #param 1 = 1.0 , param2 = 6.0

        self.publish_offboard_mode() # 설정한 값으로 오프보드 모드를 실행 

        self.publish_trajectory_setpoint(self.target[self.target_index])  #trajectory setpoint 함수에서 position p를 타겟 배열로 설정해줌 

        if self.close_enough():
            self.reached_target_counter += 1.0   #가까워지기 시작하면 1초부터 세기 시작 
            if self.reached_target_counter > self.time_threshold:
                if self.target_index < len(self.target)-1: #인덱스가 5인 부분에서 멈춰 
                    self.target_index += 1 
                    self.reached_target_counter = 0.0 # 새로운 인덱스에서 다시 0.0초로 초기화
                else:
                    self.publish_vehicle_command(VehicleCommand.VEHICLE_CMD_NAV_LAND,0.0,0.0) #마지막 웨이포인트에서 착륙 , disarm은 스스로 됨 
                    self.get_logger().info("LAND COMMAND SEND")
                    self.timer.cancel() # 타이머, 콜백 함수 종료 
                    sys.exit(0) #프로그램 정상 종료 
                    
                


def main(args=None):
    rclpy.init(args=args)
    flight_test = FlightTest()
    rclpy.spin(flight_test)
    flight_test.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()