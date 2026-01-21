#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <cv_bridge/cv_bridge.hpp>
#include <opencv2/opencv.hpp>

class GrayImageNode : public rclcpp::Node
{
public:
  GrayImageNode() : Node("gray_image_node")
  {
    sub_ = create_subscription<sensor_msgs::msg::Image>(
      "/camera/image", 10,
      std::bind(&GrayImageNode::callback, this, std::placeholders::_1));

    pub_ = create_publisher<sensor_msgs::msg::Image>(
      "/camera/image_gray", 10);
  }

private:
  void callback(const sensor_msgs::msg::Image::SharedPtr msg)
  {
    // ROS -> OpenCV
    auto cv_ptr = cv_bridge::toCvCopy(msg, "bgr8");
    cv::Mat gray;
    cv::cvtColor(cv_ptr->image, gray, cv::COLOR_BGR2GRAY);

    // OpenCV -> ROS
    auto out_msg =
      cv_bridge::CvImage(msg->header, "mono8", gray).toImageMsg();

    pub_->publish(*out_msg);
  }

  rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr sub_;
  rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr pub_;
};

int main(int argc, char **argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<GrayImageNode>());
  rclcpp::shutdown();
  return 0;
}
