#ifndef MONO_INERTIAL_HPP
#define MONO_INERTIAL_HPP

#include "ros2_orb_slam3/common.hpp"
#include "sensor_msgs/msg/imu.hpp"

class MonocularInertialMode : public rclcpp::Node
{
    public:
    std::string experimentConfig = "";
    double timeStep;
    std::string receivedConfig = "";

    MonocularInertialMode();
    ~MonocularInertialMode();
        
    private:
        std::string homeDir = "";
        std::string packagePath = "/superbuild/src/slam_pkgs/src/ros2_orb_slam3/";
        std::string OPENCV_WINDOW = "";
        std::string nodeName = "";
        std::string vocFilePath = "";
        std::string settingsFilePath = "";
        bool bSettingsFromPython = false;
        
        std::string subexperimentconfigName = "";
        std::string pubconfigackName = "";
        std::string subImgMsgName = "";
        std::string subTimestepMsgName = "";
        std::string subImuMsgName = "";

        rclcpp::Subscription<std_msgs::msg::String>::SharedPtr expConfig_subscription_;
        rclcpp::Publisher<std_msgs::msg::String>::SharedPtr configAck_publisher_;
        rclcpp::Subscription<<sensor_msgs::msg::Image>::SharedPtr subImgMsg_subscription_;
        rclcpp::Subscription<std_msgs::msg::Float64>::SharedPtr subTimestepMsg_subscription_;
        rclcpp::Subscription<<sensor_msgs::msg::Imu>::SharedPtr subImuMsg_subscription_;

        ORB_SLAM3::System* pAgent;
        ORB_SLAM3::System::eSensor sensorType;
        bool enablePangolinWindow = false;
        bool enableOpenCVWindow = false;

        std::vector<<ORB_SLAM3::IMU::Point> vImuMeas;
        std::mutex mMutexImu;

        void experimentSetting_callback(const std_msgs::msg::String& msg);
        void Timestep_callback(const std_msgs::msg::Float64& time_msg);
        void Img_callback(const sensor_msgs::msg::Image::SharedPtr msg);
        void Imu_callback(const sensor_msgs::msg::Imu::SharedPtr msg);
        void initializeVSLAM(std::string& configString);
};

#endif
