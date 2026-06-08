#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/imu.hpp"
#include "std_msgs/msg/header.hpp"
#include "px4_msgs/msg/sensor_combined.hpp"

class imu_to_vins_node : public rclcpp::Node{

    public:
        imu_to_vins_node() : Node("imu_to_vins_node"){
            std::cout << "Transit Imu data to vins" << std::endl;
            imu_publisher_ = this->create_publisher<sensor_msgs::msg::Imu>("/imu0", 10);
            
            rmw_qos_profile_t qos_profile = rmw_qos_profile_sensor_data;
		    auto qos = rclcpp::QoS(rclcpp::QoSInitialization(qos_profile.history, 5), qos_profile);

            imu_subscriptor_ = this->create_subscription<px4_msgs::msg::SensorCombined>(
                "/fmu/out/sensor_combined", qos, std::bind(
                    &imu_to_vins_node::imu_callback, this,
                    std::placeholders::_1));

        }
    private:
        rclcpp::Publisher<sensor_msgs::msg::Imu>::SharedPtr imu_publisher_;
        rclcpp::Subscription<px4_msgs::msg::SensorCombined>::SharedPtr imu_subscriptor_;

        void imu_callback(const px4_msgs::msg::SensorCombined::SharedPtr msg) const{
            sensor_msgs::msg::Imu imu_msg;
            std_msgs::msg::Header _header;
            
            _header.stamp = this->get_clock()->now();

            imu_msg.header = _header;
            std::cout << "Received imu data from px4" << std::endl;
            imu_msg.linear_acceleration.set__x(msg->accelerometer_m_s2[0]);
            imu_msg.linear_acceleration.set__y(msg->accelerometer_m_s2[1]);
            imu_msg.linear_acceleration.set__z(msg->accelerometer_m_s2[2]);

            imu_msg.angular_velocity.set__x(msg->gyro_rad[0]);
            imu_msg.angular_velocity.set__y(msg->gyro_rad[1]);
            imu_msg.angular_velocity.set__z(msg->gyro_rad[2]);

            imu_publisher_->publish(imu_msg);
        }
};

int main(int argc, char** argv){
    rclcpp::init(argc, argv);

    rclcpp::spin(std::make_shared<imu_to_vins_node>());

    rclcpp::shutdown();

    return 0;
}
