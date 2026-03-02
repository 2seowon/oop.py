import rclpy
from rclpy.node import Node
from std_msgs.msg import String

class DroneListenerNode(Node):
    def __init__(self):
        super().__init__("drone_listen_node")

        # Subscriber 생성
        self.subscription = self.create_subscription(String, "/drone_status", self.listener_callback,10)

    def listener_callback(self, msg): #callback 함수 생성
            self.get_logger().info(f"방금 드론에서 온 메시지를 읽었습니다:{msg.data}")


def main(args=None):
    rclpy.init(args=args)
    node = DroneListenerNode()

    # 데이터가 올 때까지 프로그램이 꺼지지 않고 무한정 대기하도록 쳇바퀴를 굴립니다.
    rclpy.spin(node)

    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()

# drone_listen_node가 구독하는 토픽 : /drone_status (drone_status_node)가 발행하는 토픽