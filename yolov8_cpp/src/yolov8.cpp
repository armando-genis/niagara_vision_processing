#include <rclcpp/rclcpp.hpp>

#include "sensor_msgs/msg/image.hpp"
// #include "cv_bridge/cv_bridge.h"
#include "opencv2/highgui/highgui.hpp"
#include <opencv2/dnn/dnn.hpp>
#include <opencv2/imgproc/imgproc.hpp>



#include <fstream>
#include <chrono>
#include <iomanip>

using namespace std;

const std::vector<cv::Scalar> colors = {cv::Scalar(255, 255, 0), cv::Scalar(0, 255, 0), cv::Scalar(0, 255, 255), cv::Scalar(255, 0, 0)};

const float INPUT_WIDTH = 640.0;
const float INPUT_HEIGHT = 640.0;
const float SCORE_THRESHOLD = 0.2;
const float NMS_THRESHOLD = 0.4;
const float CONFIDENCE_THRESHOLD = 0.4;

class yolov8 : public rclcpp::Node
{
private:
    /* data */
    rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr subscription_;
    void image_callback(const sensor_msgs::msg::Image::SharedPtr msg);
    void load_net(cv::dnn::Net &net, bool is_cuda);
    std::vector<std::string> load_class_list();

    bool is_cuda = false;
    std::vector<std::string> class_list;
    cv::dnn::Net net;


public:
    yolov8(/* args */);
    ~yolov8();
};

yolov8::yolov8(/* args */): Node("yolov8_node")
{
    
    subscription_ = this->create_subscription<sensor_msgs::msg::Image>(
        "/zed2/image_raw", 10, std::bind(&yolov8::image_callback, this, std::placeholders::_1));

    cv::namedWindow("Received Image", cv::WINDOW_AUTOSIZE);

    class_list = load_class_list();
    load_net(net, is_cuda);
    RCLCPP_INFO(this->get_logger(), "yolov8_node initialized");
}

yolov8::~yolov8()
{
    cv::destroyAllWindows();
}

void yolov8::image_callback(const sensor_msgs::msg::Image::SharedPtr msg){
    // Convert ROS Image message to cv::Mat manually
    int type = (msg->encoding == "mono8") ? CV_8UC1 : CV_8UC3;
    cv::Mat image(msg->height, msg->width, type, &msg->data[0]);

    // If the image is in RGB format, convert it to BGR
    if (msg->encoding == "rgb8") {
        cv::cvtColor(image, image, cv::COLOR_RGB2BGR);
    }

    // Display the image
    cv::imshow("Received Image", image);
    cv::waitKey(1);
}


std::vector<std::string> yolov8::load_class_list()
{
    std::vector<std::string> class_list;
    std::ifstream ifs("/home/genis/ros2_ws/src/niagara_vision_processing/yolov8_cpp/scripts/classes.txt");
    std::string line;
    while (getline(ifs, line))
    {
        class_list.push_back(line);
    }
    return class_list;
}


void yolov8::load_net(cv::dnn::Net &net, bool is_cuda)
{
    auto result = cv::dnn::readNet("/home/genis/ros2_ws/src/niagara_vision_processing/yolov8_cpp/scripts/yolov5s.onnx");
    if (is_cuda)
    {
        std::cout << "Attempty to use CUDA\n";
        result.setPreferableBackend(cv::dnn::DNN_BACKEND_CUDA);
        result.setPreferableTarget(cv::dnn::DNN_TARGET_CUDA_FP16);
    }
    else
    {
        std::cout << "Running on CPU\n";
        result.setPreferableBackend(cv::dnn::DNN_BACKEND_OPENCV);
        result.setPreferableTarget(cv::dnn::DNN_TARGET_CPU);
    }
    net = result;
}


int main(int argc, char** argv) {
    rclcpp::init(argc, argv);
    auto node = std::make_shared<yolov8>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}