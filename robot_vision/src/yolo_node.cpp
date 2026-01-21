#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <cv_bridge/cv_bridge.hpp>

#include <opencv2/opencv.hpp>
#include <opencv2/dnn.hpp>

class YoloNode : public rclcpp::Node
{
public:
    YoloNode() : Node("yolo_node")
    {
        // Load YOLOv8 ONNX
        net_ = cv::dnn::readNetFromONNX(
            "/home/pan/Documents/projects/robot/p4_new/ros2_ws/src/yolov8n.onnx"
        );

        net_.setPreferableBackend(cv::dnn::DNN_BACKEND_OPENCV);
        net_.setPreferableTarget(cv::dnn::DNN_TARGET_CPU);

        sub_ = this->create_subscription<sensor_msgs::msg::Image>(
            "/camera/image",
            rclcpp::SensorDataQoS(),
            std::bind(&YoloNode::imageCallback, this, std::placeholders::_1)
        );
    }

private:
    void imageCallback(const sensor_msgs::msg::Image::SharedPtr msg)
    {
        cv::Mat frame = cv_bridge::toCvCopy(msg, "bgr8")->image;

        cv::Mat blob;
        cv::dnn::blobFromImage(
            frame, blob, 1.0 / 255.0,
            cv::Size(640, 640),
            cv::Scalar(), true, false
        );

        net_.setInput(blob);
        std::vector<cv::Mat> outputs;
        net_.forward(outputs, net_.getUnconnectedOutLayersNames());

        drawDetections(frame, outputs);

        cv::imshow("YOLO C++", frame);
        cv::waitKey(1);
    }

    void drawDetections(cv::Mat &frame, const std::vector<cv::Mat> &outputs)
    {
        float confThreshold = 0.5;

        for (const auto &out : outputs)
        {
            const float *data = (float *)out.data;

            for (int i = 0; i < out.rows; ++i)
            {
                float confidence = data[4];
                if (confidence > confThreshold)
                {
                    float cx = data[0] * frame.cols;
                    float cy = data[1] * frame.rows;
                    float w  = data[2] * frame.cols;
                    float h  = data[3] * frame.rows;

                    cv::Rect box(
                        int(cx - w / 2),
                        int(cy - h / 2),
                        int(w),
                        int(h)
                    );

                    cv::rectangle(frame, box, cv::Scalar(0, 255, 0), 2);
                }
                data += out.cols;
            }
        }
    }

    rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr sub_;
    cv::dnn::Net net_;
};

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<YoloNode>());
    rclcpp::shutdown();
    return 0;
}
