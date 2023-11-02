#include <rclcpp/rclcpp.hpp>

#include "sensor_msgs/msg/image.hpp"
#include "opencv2/highgui/highgui.hpp"
#include <opencv2/dnn/dnn.hpp>
#include <opencv2/imgproc/imgproc.hpp>

#include "vision_msgs/msg/detection2_d.hpp"
#include "vision_msgs/msg/detection2_d_array.hpp"
#include "vision_msgs/msg/object_hypothesis_with_pose.hpp"

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


struct Detection
{
    int class_id;
    float confidence;
    cv::Rect box;
};


class yolov5 : public rclcpp::Node
{
private:
    /* data */
    rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr subscription_;
    rclcpp::Publisher<vision_msgs::msg::Detection2DArray>::SharedPtr publisher_;

    void image_callback(const sensor_msgs::msg::Image::SharedPtr msg);
    void load_net(cv::dnn::Net &net, bool is_cuda);
    std::vector<std::string> load_class_list();
    void detect_and_display(cv::Mat &image);
    void detect(cv::Mat &image, cv::dnn::Net &net, std::vector<Detection> &output, const std::vector<std::string> &className);
    cv::Mat format_yolov5(const cv::Mat &source);


    bool is_cuda = false;
    std::vector<std::string> class_list;
    cv::dnn::Net net;


public:
    yolov5(/* args */);
    ~yolov5();
};

yolov5::yolov5(/* args */): Node("yolov5_node")
{
    
    subscription_ = this->create_subscription<sensor_msgs::msg::Image>(
        "/zed2/image_raw", 10, std::bind(&yolov5::image_callback, this, std::placeholders::_1));
    
    publisher_ = this->create_publisher<vision_msgs::msg::Detection2DArray>("detections", 10);


    cv::namedWindow("Received Image", cv::WINDOW_AUTOSIZE);


    class_list = load_class_list();
    load_net(net, is_cuda);
    RCLCPP_INFO(this->get_logger(), "yolov5_node initialized");
}

yolov5::~yolov5()
{
    cv::destroyAllWindows();
}

void yolov5::image_callback(const sensor_msgs::msg::Image::SharedPtr msg)
{
    int type = (msg->encoding == "mono8") ? CV_8UC1 : CV_8UC3;
    cv::Mat image(msg->height, msg->width, type, &msg->data[0]);

    if (msg->encoding == "rgb8") {
        cv::cvtColor(image, image, cv::COLOR_RGB2BGR);
    }

    vision_msgs::msg::Detection2DArray detections_msg;
    detections_msg.header = msg->header; 

    std::vector<Detection> detections;
    detect(image, net, detections, class_list);
    for (auto & detection : detections) {
        vision_msgs::msg::Detection2D detection_msg;
        vision_msgs::msg::ObjectHypothesisWithPose hypothesis;

        const auto color = colors[detection.class_id % colors.size()];
        cv::rectangle(image, detection.box, color, 3);
        cv::rectangle(image, cv::Point(detection.box.x, detection.box.y - 20), cv::Point(detection.box.x + detection.box.width, detection.box.y), color, cv::FILLED);
        cv::putText(image, class_list[detection.class_id].c_str(), cv::Point(detection.box.x, detection.box.y - 5), cv::FONT_HERSHEY_SIMPLEX, 0.5, cv::Scalar(0, 0, 0));

        // Populate the bounding box information
        detection_msg.bbox.center.x = detection.box.x + detection.box.width / 2.0;
        detection_msg.bbox.center.y = detection.box.y + detection.box.height / 2.0;
        detection_msg.bbox.size_x = detection.box.width;
        detection_msg.bbox.size_y = detection.box.height;

        // Populate the class and score information
        hypothesis.id = detection.class_id;
        hypothesis.score = detection.confidence;
        detection_msg.results.push_back(hypothesis);

        // Add the detection message to the array
        detections_msg.detections.push_back(detection_msg);

    }
    publisher_->publish(detections_msg);


    cv::imshow("Received Image", image);
    cv::waitKey(1);
}

std::vector<std::string> yolov5::load_class_list()
{
    std::vector<std::string> class_list;
    std::ifstream ifs("/home/genis/ros2_ws/src/niagara_vision_processing/yolov5_cpp/yoloFiles/classes.txt");
    std::string line;
    while (getline(ifs, line))
    {
        class_list.push_back(line);
    }
    return class_list;
}


void yolov5::load_net(cv::dnn::Net &net, bool is_cuda)
{
    auto result = cv::dnn::readNet("/home/genis/ros2_ws/src/niagara_vision_processing/yolov5_cpp/yoloFiles/yolov5s.onnx");
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


void yolov5::detect_and_display(cv::Mat &image)
{


    std::vector<Detection> output;
    detect(image, net, output, class_list);

    int detections = output.size();

    for (int i = 0; i < detections; ++i)
    {
        auto detection = output[i];
        auto box = detection.box;
        auto classId = detection.class_id;
        const auto color = colors[classId % colors.size()];
        cv::rectangle(image, box, color, 3);
        cv::rectangle(image, cv::Point(box.x, box.y - 20), cv::Point(box.x + box.width, box.y), color, cv::FILLED);
        cv::putText(image, class_list[classId].c_str(), cv::Point(box.x, box.y - 5), cv::FONT_HERSHEY_SIMPLEX, 0.5, cv::Scalar(0, 0, 0));
    }
}


cv::Mat yolov5::format_yolov5(const cv::Mat &source) {
    int col = source.cols;
    int row = source.rows;
    int _max = MAX(col, row);
    cv::Mat result = cv::Mat::zeros(_max, _max, CV_8UC3);
    source.copyTo(result(cv::Rect(0, 0, col, row)));
    return result;
}

void yolov5::detect(cv::Mat &image, cv::dnn::Net &net, std::vector<Detection> &output, const std::vector<std::string> &className)
{
    cv::Mat blob;

    auto input_image = format_yolov5(image);
    
    cv::dnn::blobFromImage(input_image, blob, 1./255., cv::Size(INPUT_WIDTH, INPUT_HEIGHT), cv::Scalar(), true, false);
    net.setInput(blob);
    std::vector<cv::Mat> outputs;
    net.forward(outputs, net.getUnconnectedOutLayersNames());

    float x_factor = input_image.cols / INPUT_WIDTH;
    float y_factor = input_image.rows / INPUT_HEIGHT;
    
    float *data = (float *)outputs[0].data;

    const int rows = 25200;
    
    std::vector<int> class_ids;
    std::vector<float> confidences;
    std::vector<cv::Rect> boxes;

    for (int i = 0; i < rows; ++i) {

        float confidence = data[4];
        if (confidence >= CONFIDENCE_THRESHOLD) {

            float * classes_scores = data + 5;
            cv::Mat scores(1, className.size(), CV_32FC1, classes_scores);
            cv::Point class_id;
            double max_class_score;
            minMaxLoc(scores, 0, &max_class_score, 0, &class_id);
            if (max_class_score > SCORE_THRESHOLD) {

                confidences.push_back(confidence);

                class_ids.push_back(class_id.x);

                float x = data[0];
                float y = data[1];
                float w = data[2];
                float h = data[3];
                int left = int((x - 0.5 * w) * x_factor);
                int top = int((y - 0.5 * h) * y_factor);
                int width = int(w * x_factor);
                int height = int(h * y_factor);
                boxes.push_back(cv::Rect(left, top, width, height));
            }

        }

        data += 85;

    }
    std::vector<int> nms_result;
    cv::dnn::NMSBoxes(boxes, confidences, SCORE_THRESHOLD, NMS_THRESHOLD, nms_result);
    for (size_t i = 0; i < nms_result.size(); i++) {
        int idx = nms_result[i];
        Detection result;
        result.class_id = class_ids[idx];
        result.confidence = confidences[idx];
        result.box = boxes[idx];
        output.push_back(result);
    }
}

int main(int argc, char** argv) {
    rclcpp::init(argc, argv);
    auto node = std::make_shared<yolov5>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}