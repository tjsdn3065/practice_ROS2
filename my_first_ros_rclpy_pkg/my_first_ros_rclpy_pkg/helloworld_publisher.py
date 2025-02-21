# 첫 구절은 import 구문이다.
# rclpy의 Node 클래스를 사용하며, 퍼블리셔의 QoS 설정을 위하여 QoSProfile 클래스를 사용한다.
# 퍼블리시하는 메시지 타입은 std_msgs.msg 모듈의 String 메시지 인터페이스를 사용하기 위해 import하였다.
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile
from std_msgs.msg import String

# 이 노드의 메인 클래스는 HelloworldPublisher이고 Node 클래스를 상속해 사용할 예정이다.
class HelloworldPublisher(Node):

    def __init__(self):
        # 다음은 클래스 생성자 정의로 super()._init__(helloworld_publisher)를 이용해 부모 클래스 (Node)의 생성자를 호출하고
        # 노드 이름을 helloworld_publisher로 지정하였다.
        super().__init__('helloworld_publisher')
        # 그 다음 퍼블리셔의 QoS 설정을 위하여 QoSProfile을 호출하고 기본 depth를 10으로 설정하였다.
        # 이는 통신 상태가 원활하지 못하거나 예기치 못한 문제가 발생할 경우를 대비해
        # 퍼블리시할 데이터를 버퍼에 10개까지 저장하는 설정이다.
        qos_profile = QoSProfile(depth=10)
        # 그 다음은 Node 클래스의 create_publisher 함수를 이용해 helloworld_publisher를 설정하고 있다.
        # 매개변수로 토픽 메시지 타입은 String. 토픽 이름은 helloworld, Qos는 qos_profile이다.
        self.helloworld_publisher = self.create_publisher(String,'helloworld',qos_profile)
        # 마지막으로 Node 클래스의 create_timer 함수를 이용해 콜백 함수를 실행시킨다.
        # 첫 번째 매개변수 timer_period_sec를 1로 설정하면 1초마다 지정한 콜백 함수가 실행한다.
        # 다음 코드는 1초마다 publish_helloworld_msg 함수를 실행시킨다.
        self.timer = self.create_timer(1,self.publish_helloworld_msg)
        # count는 콜백 함수에 사용되는 카운터 값이다.
        self.count = 0

    # 다음은 앞에서 지정한 publish_helloworld_msg 콜백 함수이다.
    def publish_helloworld_msg(self):
        # 퍼블리시할 메시지는 String 타입으로 선언하였으며 실제 데이터는 msg.data 변수에 저장된다.
        msg = String()
        # 여기서는 "Hello World: 1"과 같이 매번 콜백 함수가 실행될 때마다 1씩 증가하는 count 값을 문자열에 포함시켜
        # publish 함수를 통해 퍼블리시하게 된다.
        msg.data = 'Hello World: {0}'.format(self.count)
        self.helloworld_publisher.publish(msg)
        # get_logger 함수는 터미널 창에 출력하는 함수로 로거의 종류에 따라 debug, info, warning, error, fatal을 사용한다.
        # 일반적인 정보 전달에는 info를 사용하고 있기에 info 함수를 통해 현재 퍼블리시되는 메시지를 터미널 창에 출력시킨다.
        self.get_logger().info('Published message: {0}'.format(msg.data))
        self.count += 1


def main(args=None):
    # rclpy.init를 이용해 초기화하고
    rclpy.init(args=args)
    # 앞에서 작성한 HelloworldPublisher 클래스를 node 변수로 생성한 다음
    node = HelloworldPublisher()
    # rclpy.spin 함수를 이용하여 생성한 노드를 spin 시켜 지정된 콜백 함수가 실행될 수 있도록 하고 있다.
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        # 종료(Ctrl+c)와 같은 인터럽트 시그널 예외 상황에서는 node를소멸시키고
        node.get_logger().info('Keyboard Interrupt (SIGINT)')
    finally:
        # rclpy.shutdown 함수로 노드를 종료하게 된다.
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
