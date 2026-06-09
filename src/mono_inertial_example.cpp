#include "ros2_orb_slam3/mono_inertial.hpp"

MonocularInertialMode::MonocularInertialMode() : Node("mono_inertial_node")
{
    this->declare_parameter("node_name_arg", "mono_inertial_slam_cpp");
    this->declare_parameter("settings_name", "EuRoC");

    this->nodeName = this->get_parameter("node_name_arg").as_string();
    this->experimentConfig = this->get_parameter("settings_name").as_string();

    RCLCPP_INFO(this->get_logger(), "ORB-SLAM3-V1 MONO INERTIAL NODE STARTED");
    RCLCPP_INFO(this->get_logger(), "nodeName %s", this->nodeName.c_str());

    this->homeDir = std::string(std::getenv("HOME")) + this->packagePath;
    this->vocFilePath = this->homeDir + "orb_slam3/Vocabulary/ORBvoc.txt.bin";
    RCLCPP_INFO(this->get_logger(), "voc_file %s", this->vocFilePath.c_str());

    this->subexperimentconfigName = "/mono_py_driver/experiment_settings";
    this->pubconfigackName = "/mono_py_driver/exp_settings_ack";
    this->subImgMsgName = "/mono_py_driver/img_msg";
    this->subTimestepMsgName = "/mono_py_driver/timestep_msg";
    this->subImuMsgName = "/mono_py_driver/imu_msg";

    this->expConfig_subscription_ = this->create_subscription<std_msgs::msg::String>(
        this->subexperimentconfigName, 1, 
        std::bind(&MonocularInertialMode::experimentSetting_callback, this, std::placeholders::_1));
    
    this->configAck_publisher_ = this->create_publisher<std_msgs::msg::String>(
        this->pubconfigackName, 1);
    
    this->subImgMsg_subscription_ = this->create_subscription<<sensor_msgs::msg::Image>(
        this->subImgMsgName, 1,
        std::bind(&MonocularInertialMode::Img_callback, this, std::placeholders::_1));
    
    this->subTimestepMsg_subscription_ = this->create_subscription<std_msgs::msg::Float64>(
        this->subTimestepMsgName, 1,
        std::bind(&MonocularInertialMode::Timestep_callback, this, std::placeholders::_1));
    
    this->subImuMsg_subscription_ = this->create_subscription<<sensor_msgs::msg::Imu>(
        this->subImuMsgName, 100,
        std::bind(&MonocularInertialMode::Imu_callback, this, std::placeholders::_1));

    RCLCPP_INFO(this->get_logger(), "Waiting to finish handshake ......");
}

MonocularInertialMode::~MonocularInertialMode()
{
    if (this->pAgent != nullptr)
    {
        this->pAgent->Shutdown();
        delete this->pAgent;
    }
}

void MonocularInertialMode::experimentSetting_callback(const std_msgs::msg::String& msg)
{
    if (!this->bSettingsFromPython)
    {
        this->experimentConfig = msg.data;
        RCLCPP_INFO(this->get_logger(), "Configuration YAML file name: %s", this->experimentConfig.c_str());
        
        this->settingsFilePath = this->homeDir + "orb_slam3/config/Monocular-Inertial/" + this->experimentConfig + ".yaml";
        RCLCPP_INFO(this->get_logger(), "Path to settings file: %s", this->settingsFilePath.c_str());

        this->initializeVSLAM(this->experimentConfig);
        
        std_msgs::msg::String ackMsg;
        ackMsg.data = "ACK";
        this->configAck_publisher_->publish(ackMsg);
        
        RCLCPP_INFO(this->get_logger(), "Sent response: %s", ackMsg.data.c_str());
        this->bSettingsFromPython = true;
    }
}

void MonocularInertialMode::initializeVSLAM(std::string& configString)
{
    this->sensorType = ORB_SLAM3::System::IMU_MONOCULAR;
    this->pAgent = new ORB_SLAM3::System(
        this->vocFilePath, 
        this->settingsFilePath, 
        this->sensorType, 
        this->enablePangolinWindow
    );
}

void MonocularInertialMode::Timestep_callback(const std_msgs::msg::Float64& time_msg)
{
    this->timeStep = time_msg.data;
}

void MonocularInertialMode::Imu_callback(const sensor_msgs::msg::Imu::SharedPtr msg)
{
    std::lock_guard<std::mutex> lock(mMutexImu);
    
    double timestamp = msg->header.stamp.sec + msg->header.stamp.nanosec * 1e-9;
    
    ORB_SLAM3::IMU::Point imuPoint(
        msg->linear_acceleration.x, 
        msg->linear_acceleration.y, 
        msg->linear_acceleration.z,
        msg->angular_velocity.x, 
        msg->angular_velocity.y, 
        msg->angular_velocity.z,
        timestamp
    );
    
    vImuMeas.push_back(imuPoint);
}

void MonocularInertialMode::Img_callback(const sensor_msgs::msg::Image::SharedPtr msg)
{
    if (!this->bSettingsFromPython)
    {
        RCLCPP_WARN(this->get_logger(), "Image received before settings configured");
        return;
    }

    cv_bridge::CvImageConstPtr cv_ptr;
    try
    {
        cv_ptr = cv_bridge::toCvShare(msg, sensor_msgs::image_encodings::BGR8);
    }
    catch (cv_bridge::Exception& e)
    {
        RCLCPP_ERROR(this->get_logger(), "cv_bridge exception: %s", e.what());
        return;
    }

    cv::Mat img = cv_ptr->image;
    if (img.empty())
    {
        RCLCPP_WARN(this->get_logger(), "Empty image received");
        return;
    }

    cv::Mat gray;
    cv::cvtColor(img, gray, cv::COLOR_BGR2GRAY);

    std::vector<<ORB_SLAM3::IMU::Point> vImuMeasCopy;
    {
        std::lock_guard<std::mutex> lock(mMutexImu);
        vImuMeasCopy = vImuMeas;
        vImuMeas.clear();
    }

    this->pAgent->TrackMonocular(gray, this->timeStep, vImuMeasCopy);
}

int main(int argc, char** argv)
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<<MonocularInertialMode>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}
