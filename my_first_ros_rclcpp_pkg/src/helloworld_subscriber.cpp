#include <functional>
#include <memory>

#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"

// placeholders 클래스는 bind 함수의 대체자 역할을 위하여 _1로 선언하였다.
using std::placeholders::_1;

class HelloworldSubscriber : public rclcpp::Node
{
public:
  HelloworldSubscriber()
  : Node("Helloworld_subscriber")
  {
    auto qos_profile = rclcpp::QoS(rclcpp::KeepLast(10));
    // 그 다음으로는 Node 클래스의 create_subscription 함수를 이용해 서브스크라이버를 설정하고 있다.
    // 매개변수의 토픽 메시지 타입은 String, 토픽 이름은 helloworld, QoS 설정은 qos_profile, 콜백 함수는 subscribe_topic_message로 설정하였다.
    helloworld_subscriber_ = this->create_subscription<std_msgs::msg::String>(
      "helloworld",
      qos_profile,
      std::bind(&HelloworldSubscriber::subscribe_topic_message, this, _1));
  }

private:
  // 다음은 앞에서 지정한 subscribe_topic_message 콜백 함수이다.
  // 서브스크라이브한 메시지는 String 타입이고 msg 인자를 통해 받은 메시지는 msg.data 변수에 저장된다.
  void subscribe_topic_message(const std_msgs::msg::String::SharedPtr msg) const
  {
    RCLCPP_INFO(this->get_logger(), "Received message: '%s'", msg->data.c_str());
  }
  // 함수 바깥에는 클래스에서 private 변수로 사용되는 helloworld_subscriber_를 선언하였다.
  rclcpp::Subscription<std_msgs::msg::String>::SharedPtr helloworld_subscriber_;
};


int main(int argc, char * argv[])
{
  rclcpp::init(argc,argv);
  auto node = std::make_shared<HelloworldSubscriber>();
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}
