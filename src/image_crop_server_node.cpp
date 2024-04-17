#include <filesystem>
#include <cv_bridge/cv_bridge.h>
#include <sstream> 
#include <opencv2/opencv.hpp>
#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/header.hpp"
#include "image_processing_pkg/msg/target_cropped_axis.hpp"
#include "image_processing_pkg/srv/image_crop.hpp"

using namespace std;
using std::placeholders::_1;
using std::placeholders::_2;

class ImageCropServer : public rclcpp::Node
{    
    float x_axis = 0; 
    float y_axis = 0;
    public:    
    ImageCropServer()
    : Node("image_crop_server_node")
    {     
        RCLCPP_INFO(this->get_logger(), "Starting image crop server");        
        subscription_ = this->create_subscription<image_processing_pkg::msg::TargetCroppedAxis>(
        "target_cropped_axis", 10, std::bind(&ImageCropServer::topic_callback, this, _1));
        service_ = this->create_service<image_processing_pkg::srv::ImageCrop>("process_image", std::bind(&ImageCropServer::process_image, this, _1, _2));
    }

    void topic_callback(const image_processing_pkg::msg::TargetCroppedAxis & msg)
    {
      RCLCPP_INFO(this->get_logger(), "Axis received X:'%f' and Y:'%f'", msg.x, msg.y);
      x_axis= msg.x;
      y_axis = msg.y;
    }

    void process_image(const image_processing_pkg::srv::ImageCrop::Request::SharedPtr request, 
    image_processing_pkg::srv::ImageCrop::Response::SharedPtr response)
    {
        RCLCPP_INFO(this->get_logger(), "Incoming request id '%d' Image '%s'",request->request_id, request->image_path.c_str());

        string root_path = std::filesystem::current_path(); 
        string image_path = request->image_path;
        if(image_path.empty()){
            image_path = root_path + "/src/image_processing_pkg/src/images/image_1.png";
        }

        cv::Mat img = cv::imread(image_path);        
        RCLCPP_INFO(this->get_logger(), "Image Width: '%s'", std::to_string(img.size().width).c_str());
        RCLCPP_INFO(this->get_logger(), "Image Height: '%s'", std::to_string(img.size().height).c_str());

        
        // Crop image parameters (crop image always from the center point)
        if(x_axis > 0 && y_axis > 0){
            int offset_x = img.size().width/2;
            int offset_y = img.size().height/2;
            int cropped_width = img.size().width * x_axis;
            int cropped_height = img.size().height * y_axis;
            int final_offset_x = std::max(0, offset_x - cropped_width/2);
            int final_offset_y = std::max(0, offset_y - cropped_height/2);

            RCLCPP_INFO(this->get_logger(), "Offset X: '%s'", std::to_string(final_offset_x).c_str());
            RCLCPP_INFO(this->get_logger(), "Offset Y: '%s'", std::to_string(final_offset_y).c_str());
            RCLCPP_INFO(this->get_logger(), "Height to crop: '%s'", std::to_string(cropped_height).c_str());
            RCLCPP_INFO(this->get_logger(), "Width to crop: '%s'", std::to_string(cropped_width).c_str());

            cv::Rect r(final_offset_x,final_offset_y,cropped_width,cropped_height);
            cv::Mat cropped_image = img(r);
            
            //display image (debug)
            //cv::imshow("Original Image", img);        
            //cv::imshow("Cropped Image", cropped_image);
            //cv::waitKey(0);            
            msg_ = cv_bridge::CvImage(std_msgs::msg::Header(), "bgr8", cropped_image).toImageMsg();
            
            response->image = *msg_.get();
            RCLCPP_INFO(this->get_logger(), "Response sent: Finishing process");
            
        }       
    }

    private:        
        sensor_msgs::msg::Image::SharedPtr msg_;
        rclcpp::Subscription<image_processing_pkg::msg::TargetCroppedAxis>::SharedPtr subscription_;
        rclcpp::Service<image_processing_pkg::srv::ImageCrop>::SharedPtr service_;
};

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<ImageCropServer>());
  rclcpp::shutdown();
  return 0;
}