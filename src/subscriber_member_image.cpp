#include <memory>

#include "rclcpp/rclcpp.hpp"
#include "image_processing_pkg/msg/target_cropped_axis.hpp"

using std::placeholders::_1;

class MinimalSubscriber : public rclcpp::Node
{
  public:
    MinimalSubscriber()
    : Node("minimal_subscriber")
    {
      subscription_ = this->create_subscription<image_processing_pkg::msg::TargetCroppedAxis>(
      "custom", 10, std::bind(&MinimalSubscriber::topic_callback, this, _1));
    }

  private:
    void topic_callback(const image_processing_pkg::msg::TargetCroppedAxis & msg) const
    {
      RCLCPP_INFO(this->get_logger(), "I heard: '%f' and '%f'", msg.x, msg.y);
    }
    rclcpp::Subscription<image_processing_pkg::msg::TargetCroppedAxis>::SharedPtr subscription_;
};

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<MinimalSubscriber>());
  rclcpp::shutdown();
  return 0;
}