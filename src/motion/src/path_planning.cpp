#include <memory>
#include <functional>

#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/bool.hpp"
#include "interfaces/msg/start_end_points.hpp"
#include "interfaces/srv/arm_pose.hpp"
#include "interfaces/srv/arduino_call.hpp"

struct Pose {
    Pose() = default;
    Pose(float px, float py, float pz, float pyaw) : x{ px }, y{ py }, z{ pz }, yaw{ pyaw } {}
    float x;
    float y;
    float z;
    float yaw;
};

class PathPlanning : public rclcpp::Node
{
    public:
        PathPlanning() : Node("path_planning")
        {
            // Create a timer 
            timer_ = this->create_wall_timer(std::chrono::milliseconds(1000), std::bind(&PathPlanning::timerCallback, this));
            // Create a publisher to publish the robot state
            motionStatePublisher_ = this->create_publisher<std_msgs::msg::Bool>("motion_state", 10);
            // Create a subscriber
            startEndPointsSub_ = this->create_subscription<interfaces::msg::StartEndPoints>("start_end_points", 10, std::bind(&PathPlanning::subscriberCallback, this, std::placeholders::_1));
            // Create arm movement client
            armPoseClient_ = create_client<interfaces::srv::ArmPose>("arm_pose");
            while (!armPoseClient_->wait_for_service(std::chrono::seconds(1)))
            {
                RCLCPP_INFO(get_logger(), "Arm pose service not available, waiting...");
            }
            // Create arduino call client
            arduinoCallClient_ = create_client<interfaces::srv::ArduinoCall>("arduino_service");
            while (!arduinoCallClient_->wait_for_service(std::chrono::seconds(1)))
            {
                RCLCPP_INFO(get_logger(), "Arduino call service not available, waiting...");
            }

            // Initialise variables
            home_ = Pose(0.0, 0.5, 0.2, 0.0);
            isBusy_ = false;
            dropHeight_ = 0.05;
            pathPointsIndex_ = 0;
        }

    private:

        void timerCallback()
        {
            std_msgs::msg::Bool isMoving;
            if (isBusy_) {
                isMoving.data = true;
            }
            else {
                isMoving.data = false;
            }
            motionStatePublisher_->publish(isMoving);
        }

        void subscriberCallback(const interfaces::msg::StartEndPoints& msg) 
        {
            if (isBusy_) return;
            isBusy_ = true;
            pathPoints_.clear();

            // Construct path points
            pathPoints_.push_back(Pose(msg.start_x, msg.start_y, msg.start_z + dropHeight_, msg.start_yaw));
            pathPoints_.push_back(Pose(msg.start_x, msg.start_y, msg.start_z, msg.start_yaw));
            pathPoints_.push_back(Pose(msg.start_x, msg.start_y, msg.start_z + dropHeight_, msg.start_yaw));
            pathPoints_.push_back(Pose(msg.end_x, msg.end_y, msg.end_z + dropHeight_, msg.end_yaw));
            pathPoints_.push_back(Pose(msg.end_x, msg.end_y, msg.end_z, msg.end_yaw));
            pathPoints_.push_back(Pose(msg.end_x, msg.end_y, msg.end_z + dropHeight_, msg.end_yaw));
            pathPoints_.push_back(home_);

            // Call service
            callArmPoseService(pathPoints_[pathPointsIndex_]);
        }

        void callArmPoseService(const Pose& pose)
        {
            auto request = std::make_shared<interfaces::srv::ArmPose::Request>();
            request->x = pose.x;
            request->y = pose.y;
            request->z = pose.z;
            request->yaw = pose.yaw;

            auto future_result = armPoseClient_->async_send_request(
                request,
                std::bind(&PathPlanning::handleArmPoseResponse, this, std::placeholders::_1)
            );

            RCLCPP_INFO(get_logger(), "Phase: %d", pathPointsIndex_);
        }

        void handleArmPoseResponse(rclcpp::Client<interfaces::srv::ArmPose>::SharedFuture future)
        {
            if (future.get()) {
                if (future.get()->success) {    
                    RCLCPP_INFO(get_logger(), "Successfully reached pose.");

                    pathPointsIndex_++;
                    if (pathPointsIndex_ < pathPoints_.size()) {
                        if (pathPointsIndex_ == 2) {
                            // Pick up block
                            callArduinoService("Free Brick Position Reached");
                        }
                        else if (pathPointsIndex_ == 5) {
                            // Drop the block
                            callArduinoService("Target Brick Position Reached");
                        }
                        else {
                            // Move to pose
                            callArmPoseService(pathPoints_[pathPointsIndex_]);
                        }
                    }
                    else {
                        RCLCPP_INFO(get_logger(), "Path completed!");
                        // Reset node
                        pathPointsIndex_ = 0;
                        isBusy_ = false;
                    }
                    
                } else {
                    RCLCPP_ERROR(get_logger(), "Planning failed!");
                    // Reset node and go back to home position
                    pathPointsIndex_ = pathPoints_.size() - 1;
                    callArmPoseService(pathPoints_[pathPointsIndex_]);
                    rclcpp::sleep_for(std::chrono::seconds(5));
                    isBusy_ = false;
                }   
            } else {
                RCLCPP_ERROR(get_logger(), "Arm service call failed!");
            }

        }

        void callArduinoService(std::string commandStr) 
        {
            //callArmPoseService(pathPoints_[pathPointsIndex_]);
            auto request = std::make_shared<interfaces::srv::ArduinoCall::Request>();
            request->command = commandStr;

            auto future_result = arduinoCallClient_->async_send_request(
                request,
                std::bind(&PathPlanning::handleArduinoResponse, this, std::placeholders::_1)
            );
        }

        void handleArduinoResponse(rclcpp::Client<interfaces::srv::ArduinoCall>::SharedFuture future)
        {
            if (future.get()->response == "Invalid command received." || future.get()->response == "No valid data received from the Arduino.") {
                RCLCPP_ERROR(get_logger(), "Arduino state error");
            } else {
                callArmPoseService(pathPoints_[pathPointsIndex_]);
            }

        }

        rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr motionStatePublisher_;
        rclcpp::Subscription<interfaces::msg::StartEndPoints>::SharedPtr startEndPointsSub_;
        rclcpp::Client<interfaces::srv::ArmPose>::SharedPtr armPoseClient_;
        rclcpp::Client<interfaces::srv::ArduinoCall>::SharedPtr arduinoCallClient_;
        rclcpp::TimerBase::SharedPtr timer_;
        Pose home_;
        bool isBusy_;
        float dropHeight_;
        uint8_t pathPointsIndex_;
        std::vector<Pose> pathPoints_;
};

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<PathPlanning>());
    rclcpp::shutdown();
    return 0;
}
