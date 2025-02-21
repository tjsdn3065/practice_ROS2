// 첫 구절은 include 및 namespace 구문이다.
// 코드에서 사용되는 std 계열의 헤더를 우선 선언하고 있으며,
#include <chrono>
#include <functional>
#include <memory>
#include <string>

// 이어서 rclcpp의 Node 클래스를 사용하기 위한 rclcpp.hpp 헤더 파일과
// 퍼블리시하는 메시지의 타입인 String 메시지 인터페이스를 사용하기 위해 string.hpp 헤더 파일을 포함시켰다.
#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"

// chrono_literals는 추후에 "500ms", "1s"와 같이 시간을 가식성이 높은 문자로 표현하기 위해 namespace를 사용할 수 있도록 선언하였다.
using namespace std::chrono_literals;


// 이 노드의 메인 클래스는 HelloworldPublisher로 rclcpp의 Node 클래스를 상속해 사용할 예정이다.
class HelloworldPublisher : public rclcpp::Node
{
public:
  // 다음은 클래스 생성자의 정의로 Node("helloworld_publisher"), count_(0)와 같이 부모 클래스(Node)의 생성자를 호출하고
  // 노드 이름을 helloworld_publisher로 지정하였다.
  // count_ 변수는 0으로 초기화한다.
  HelloworldPublisher()
  : Node("helloworld_publisher"), count_(0)
  {
    // 그 다음 퍼블리셔의 QoS 설정을 위하여 rclcpp::Qos(rclcpp::KeepLast(10))과 같이 기본 QoS에서 Keeplast 형태로 depth를 10으로 하자.
    // 이는 통신 상태가 원활하지 못하거나 예기치 못한 상황이 발생할 경우 퍼블리시할 데이터를 버퍼에 10개까지 저장하는 설정이다.
    auto qos_profile = rclcpp::QoS(rclcpp::KeepLast(10));
    // 그 다음으로는 Node 클래스의 create_publisher 함수를 이용하여 퍼블리셔를 설정하고 있다.
    // 매개변수의 토픽 메시지 타입은 String, 토픽 이름은 helloworld, Qos는 qos_profile로 설정했다.
    helloworld_publisher_ = this->create_publisher<std_msgs::msg::String>(
      "helloworld", qos_profile);
    // 마지막으로 Node 클래스의 create_wall_timer 함수를 이용해 콜백 함수를 실행시킨다.
    // 첫 번째 매개변수는 주기이며 1s로 설정하면 1초마다 지정한 콜백 함수가 실행된다.
    // 다음 코드는 1초마다 publish_helloworld_msg 함수를 실행시킨다.
    timer_ = this->create_wall_timer(
      1s, std::bind(&HelloworldPublisher::publish_helloworld_msg, this));
  }

private:
  // 다음은 앞에서 지정한 publish_helloworld_msg 콜백 함수이다.
  void publish_helloworld_msg()
  {
    // 퍼블리시할 메시지는 String 타입으로 선언하였고, 실제 데이터는 msg.data 변수에 저장하게 된다.
    auto msg = std_msgs::msg::String();
    // 여기서는 "Hello World: 1"과 같이 매번 콜백 함수가 실행될 때마다 1씩 증가하는 count_ 변숫값을 문자열에 포함시켜
    // publish 함수를 통해 퍼블리시하게 된다.
    msg.data = "Hello World: " + std::to_string(count_++);
    helloworld_publisher_->publish(msg);
    // RCLCPP_INFO 함수는 터미널 창에 출력하는 함수로
    // 로거의 종류에 따라 RCLCPP_DEBUG, RCLCPP_INFO, RCLCPP_WARN, RCLCPP_ERROR, RCLCPP_FATAL을 선택해 사용하면 된다.
    RCLCPP_INFO(this->get_logger(), "Published message: '%s'", msg.data.c_str());
  }
  // 마지막으로 클래스에서 private 변수로 사용되는 timer_, helloworld_publisher, count_를 선언하였다.
  rclcpp::TimerBase::SharedPtr timer_;
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr helloworld_publisher_;
  size_t count_;
};


// 마지막은 main 함수로
int main(int argc, char * argv[])
{
  // rclcpp::init를 이용하여 초기화하고
  rclcpp::init(argc,argv);
  // 앞에서 작성한 HelloworldPublisher클래스를 node 변수로 생성한 다음
  auto node = std::make_shared<HelloworldPublisher>();
  // rclpyspin 함수를 이용하여 생성한 노드를 spin시켜 지정된 콜백 함수가 실행될 수 있도록 하고 있다.
  rclcpp::spin(node);
  // 종료(Ctrl+c)와 같은 인터럽트 시그널 예외 상황에서는 rclcpp::shutdown 함수로 노드를 소멸하고 프로세스를 종료하게 된다.
  rclcpp::shutdown();
  return 0;
}
