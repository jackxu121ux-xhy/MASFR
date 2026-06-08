#include "rclcpp/rclcpp.hpp"
#include "opencv2/opencv.hpp"

#include "sensor_msgs/msg/image.hpp"
#include "cv_bridge/cv_bridge.hpp"
#include <memory>

class img_to_vins_node : public rclcpp::Node{

    public:
        img_to_vins_node() : Node("sensor_to_vins_node"){
            std::cout << "Transit Camera data to vins" << std::endl;
            cv::VideoCapture cap = cv::VideoCapture(0);
            if(!cap.isOpened()){
                std::cout << "Failed to open camera!" << std::endl;
                return;
            }
            cap.set(cv::CAP_PROP_FRAME_HEIGHT, 720);
            cap.set(cv::CAP_PROP_FRAME_WIDTH, 2560);

            cv::Mat frame;
            cv::Mat frame_left;
            cv::Mat frame_right;

            left_publisher_ = this->create_publisher<sensor_msgs::msg::Image>("/cam0/image_raw", 10);
            right_publisher_ = this->create_publisher<sensor_msgs::msg::Image>("/cam1/image_raw", 10);

            while(rclcpp::ok()){
                bool ret = cap.read(frame);
                if(!ret){
                   break; 
                }
                frame_left = frame(cv::Rect(0, 0, 1280, 720));
                frame_right = frame(cv::Rect(1280, 0, 1280, 720));
                publish_left_image(frame_left);
                publish_right_image(frame_right);
            }
            

        }
    private:
        rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr left_publisher_;
        rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr right_publisher_;
        

        void publish_left_image(cv::Mat image){
            std_msgs::msg::Header _header;
            sensor_msgs::msg::Image _img;
            _header.stamp = this->get_clock() -> now();

            cv_bridge::CvImage _image_cvb_= cv_bridge::CvImage(
                _header, sensor_msgs::image_encodings::BGR8, image);
            _image_cvb_.toImageMsg(_img);

            left_publisher_->publish(_img);
        }

        void publish_right_image(cv::Mat image){
            std_msgs::msg::Header _header;
            sensor_msgs::msg::Image _img;
            _header.stamp = this->get_clock() -> now();

            cv_bridge::CvImage _image_cvb_= cv_bridge::CvImage(
                _header, sensor_msgs::image_encodings::BGR8, image);
            _image_cvb_.toImageMsg(_img);

            right_publisher_->publish(_img);
        }
};

int main(int argc, char** argv){
    rclcpp::init(argc, argv);

    rclcpp::spin(std::make_shared<img_to_vins_node>());

    rclcpp::shutdown();

    return 0;
}
