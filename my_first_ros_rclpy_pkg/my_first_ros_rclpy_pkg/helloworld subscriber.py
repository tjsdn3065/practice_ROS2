import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile
from std_msgs.msg import String

# 이 노드의 메인 클래스는 HelloworldSubscriber로 Node 클래스를 상속해 사용할 예정이다.
class HelloworldSubscriber(Node):

    def __init__(self):
        # 다음은 클래스 생성자의 정의로 super().__init__('Helloworld_subscriber)를 이용해
        # 부모 클래스(Node)의 생성자를 호출하고 노드 이름을 Helloworld_subscriber로 지정하였다.
        super().__init__('Helloworld_subscriber')
        qos_profile = QoSProfile(depth=10)
        self.helloworld_subscriber = self.create_subscription(
            String,
            'helloworld',
            self.subscribe_topic_message,
            qos_profile)

    # 다음은 앞에서 지정한 subscribe_topic_message 콜백 함수이다.
    # 서브스크라이브한 메시지는 String 타입이며, 실제 데이터는 msg.data 변수에 저장되어 있다.
    def subscribe_topic_message(self, msg):
        self.get_logger().info('Received message: {0}'.format(msg.data))


def main(args=None):
    rclpy.init(args=args)
    node = HelloworldSubscriber()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info('Keyboard Interrupt (SIGINT)')
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
