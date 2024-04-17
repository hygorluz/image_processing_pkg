#include <chrono>
#include <sstream>
#include <filesystem> 
#include <cv_bridge/cv_bridge.h>
#include <opencv2/opencv.hpp>
#include <unistd.h>
#include <regex>
#include "rclcpp/rclcpp.hpp" 
#include "image_processing_pkg/msg/target_cropped_axis.hpp"
#include "image_processing_pkg/srv/image_crop.hpp"

using namespace std;
using namespace std::chrono;

using std::placeholders::_1;
using std::placeholders::_2;

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  // Node, Client and Publisher
  auto node = rclcpp::Node::make_shared("image_crop_client_node");
  auto client = node->create_client<image_processing_pkg::srv::ImageCrop>("process_image");
  auto publisher = node->create_publisher<image_processing_pkg::msg::TargetCroppedAxis>("target_cropped_axis", 10);

  // Parameters
  string root_path = std::filesystem::current_path();
  root_path = std::regex_replace(root_path, std::regex("\\/launch"), "");
  string image_path =  root_path + "/src/image_processing_pkg/res/images/";
  node->declare_parameter("interation_num", 3);
  node->declare_parameter("cropped_image_path", root_path + "/src/image_processing_pkg/res/cropped");
  node->declare_parameter("image_name", "image_1.jpeg");
  auto request = std::make_shared<image_processing_pkg::srv::ImageCrop::Request>();
  
  string cropped_image_path = node->get_parameter("cropped_image_path").as_string();
  int interation_num = node->get_parameter("interation_num").as_int();
  if(interation_num <= 0){
    interation_num = 3;
  }

  while (!client->wait_for_service(1s)) {
    if (!rclcpp::ok()) {
      RCLCPP_ERROR(node->get_logger(), "Interrupted while waiting for the service. Exiting.");
      return 0;
    }
    RCLCPP_INFO(node->get_logger(), "service not available, waiting again...");
  }

  for (int i = 1;i<=interation_num;i++){

    request->image_path =  image_path + node->get_parameter("image_name").as_string();
    request->request_id = i;
    if (request->image_path.empty()) {
      RCLCPP_INFO(node->get_logger(), "Image path can not be NULL - image_crop_client_node");
      return 1;
    }

    RCLCPP_INFO(node->get_logger(), "Request '%d'- Image : '%s'", i, request->image_path.c_str());        
    auto target_cropped_axis = image_processing_pkg::msg::TargetCroppedAxis();
    float proportion = std::min(1.0, (1.0/interation_num)*i);
    int proportion_percent = proportion * 100; 
    target_cropped_axis.x = proportion;
    target_cropped_axis.y = proportion;
    RCLCPP_INFO(node->get_logger(), "Publishing Axis X:'%f'- Y:'%f'", target_cropped_axis.x, target_cropped_axis.y);
    publisher->publish(target_cropped_axis);
    sleep(0.5);

    // Send request to server
    auto result = client->async_send_request(request);
    if (rclcpp::spin_until_future_complete(node, result) == rclcpp::FutureReturnCode::SUCCESS)
    {
      RCLCPP_INFO(node->get_logger(), "Image crop complete");
      RCLCPP_INFO(node->get_logger(), "Starting save proccess...");
      sensor_msgs::msg::Image image = result.get()->image;

      //Folder creation and Save cropped image
      stringstream ss;
      string folderCreateCommand = "mkdir -p " + cropped_image_path;
      system(folderCreateCommand.c_str());
      milliseconds ms = duration_cast< milliseconds >(
        system_clock::now().time_since_epoch()
      );
      ss<<cropped_image_path<<"/"<<"img_proportion_"<<proportion_percent<<"_percent_"<<ms.count()<<".jpg";
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

  rclcpp::shutdown();
  return 0;
}
