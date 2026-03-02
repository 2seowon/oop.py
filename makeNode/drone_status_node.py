import rclpy
from rclpy.node import Node
from std_msgs.msg import String

# ROS2 Node Class를 상속받아 새로운 class 정의
class DroneStatusNode(Node):
    def __init__(self):
        super().__init__('drone_status_node') #노드 이름을 지어줌 (자식 클래스)

        # Publisher : String 타입의 메시지를 '/drone_status' 라는 토픽으로 발행
        self.publisher_ = self.create_publisher(String, '/drone_status', 10)

        # 타이머 생성 : 1.0초마다 timer_callback 함수를 실행 (특정 함수를 반복 실행하는 타이머 생성)
        timer_period = 1.0
        self.timer = self.create_timer(timer_period, self.timer_callback)

    def timer_callback(self):
        msg = String()
        msg.data = 'drone_status'

        #메시지 객체를 실제로 토픽에 발행
        self.publisher_.publish(msg) #publisher_라는 토픽에 msg를 발행
        # 터미널 창에 해당 메시지를 로그로 띄움
        self.get_logger().info(f"Publishing : {msg.data}")


def main(args=None):
    rclpy.init(args=args)
    node = DroneStatusNode()

    # 노드가 종료되지 않고 계속 콜백 함수를 실행할 수 있도록 붙잡아둡니다.
    rclpy.spin(node)

    # 종료 시 메모리 정리
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
