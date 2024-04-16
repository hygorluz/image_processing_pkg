#include <chrono>
#include <memory>
#include <sstream>
#include <filesystem> 
#include <cv_bridge/cv_bridge.h>
#include <opencv2/opencv.hpp>
#include<unistd.h> 
#include "rclcpp/rclcpp.hpp" 
#include "image_processing_pkg/msg/target_cropped_axis.hpp"
#include "image_processing_pkg/srv/image_crop.hpp"

using namespace std;
using namespace std::chrono;

using std::placeholders::_1;
using std::placeholders::_2;

class ImageCropClient : public rclcpp::Node
{
  public:
    void timer_callback()
    {
      auto message = image_processing_pkg::msg::TargetCroppedAxis();
      message.x = 0.5;
      message.y = 0.5;
      RCLCPP_INFO_STREAM(this->get_logger(), "Publishing: '" << message.x << "' and '" << message.y);
      //publisher->publish(message);
    }  
};

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
 
  auto node = rclcpp::Node::make_shared("image_crop_client_node");
  auto client = node->create_client<image_processing_pkg::srv::ImageCrop>("process_image");
  auto publisher = node->create_publisher<image_processing_pkg::msg::TargetCroppedAxis>("target_cropped_axis", 10);
  string root_path = std::filesystem::current_path(); 
  node->declare_parameter("tries", 3);
  node->declare_parameter("cropped_image_path", root_path + "/src/image_processing_pkg/src/cropped/");
  auto request = std::make_shared<image_processing_pkg::srv::ImageCrop::Request>();
  
  string cropped_image_path = node->get_parameter("cropped_image_path").as_string();
  int tries = node->get_parameter("tries").as_int();

  while (!client->wait_for_service(1s)) {
    if (!rclcpp::ok()) {
      RCLCPP_ERROR(node->get_logger(), "Interrupted while waiting for the service. Exiting.");
      return 0;
    }
    RCLCPP_INFO(node->get_logger(), "service not available, waiting again...");
  }
  for (int i = 1;i<=tries;i++){
    request->image_path = "/home/hygor/ros2_ws/src/image_processing_pkg/src/images/image_1.png";
    request->request_id = i;
    RCLCPP_INFO(node->get_logger(), "Request '%d'- Image : '%s'", i, request->image_path.c_str());    
    auto target_cropped_axis = image_processing_pkg::msg::TargetCroppedAxis();
    target_cropped_axis.x = i * 0.25;
    target_cropped_axis.y = i * 0.25;
    publisher->publish(target_cropped_axis);
    sleep(0.5);
    auto result = client->async_send_request(request);

    if (rclcpp::spin_until_future_complete(node, result) == rclcpp::FutureReturnCode::SUCCESS)
    {
      RCLCPP_INFO(node->get_logger(), "Image crop complete");
      RCLCPP_INFO(node->get_logger(), "Starting save proccess...");
      sensor_msgs::msg::Image image = result.get()->image;
      //Save the cropped Image
      stringstream ss;
      string folderCreateCommand = "mkdir -p " + cropped_image_path;
      system(folderCreateCommand.c_str());
      milliseconds ms = duration_cast< milliseconds >(
      system_clock::now().time_since_epoch()
      );
      ss<<cropped_image_path<<"/"<<"cropped_image_"<<i<<"_"<<ms.count()<<".jpg";
      string fullPath = ss.str();
      ss.str("");
      
      cv_bridge::CvImagePtr cvImage = cv_bridge::toCvCopy(image);
      cv::imwrite(fullPath, cvImage->image);
      RCLCPP_INFO(node->get_logger(), "Image: '%s' Saved!", fullPath.c_str());
      sleep(1);
    } else {
      RCLCPP_ERROR(node->get_logger(), "Failed to call service image_crop_server_node");
    }
  }  

  //rclcpp::spin(std::make_shared<ImageCropClient>());
  rclcpp::shutdown();
  return 0;
}
