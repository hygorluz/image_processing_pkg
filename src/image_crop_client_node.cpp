#include <chrono>
#include <memory>
#include "rclcpp/rclcpp.hpp" 
#include "image_processing_pkg/msg/target_cropped_axis.hpp"

using namespace std;
using namespace std::chrono_literals;

class ImageCropClient : public rclcpp::Node
{
  public:
    ImageCropClient()
    : Node("image_crop_client_node")
    {
      publisher_ = this->create_publisher<image_processing_pkg::msg::TargetCroppedAxis>("target_cropped_axis", 10);
      timer_ = this->create_wall_timer(
        1000ms, std::bind(&ImageCropClient::timer_callback, this));
    }

  private:
    void timer_callback()
    {

      auto message = image_processing_pkg::msg::TargetCroppedAxis();
      message.x = 0.5;
      message.y = 0.5;
      RCLCPP_INFO_STREAM(this->get_logger(), "Publishing: '" << message.x << "' and '" << message.y);

      publisher_->publish(message);
    }

    rclcpp::TimerBase::SharedPtr timer_;
    rclcpp::Publisher<image_processing_pkg::msg::TargetCroppedAxis>::SharedPtr publisher_;
  
};

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<ImageCropClient>());
  rclcpp::shutdown();
  return 0;
}
