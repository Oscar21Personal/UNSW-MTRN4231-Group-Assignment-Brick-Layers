#include <memory>
#include <functional>

#include "rclcpp/rclcpp.hpp"
#include "tf2/exceptions.h"
#include "tf2/LinearMath/Quaternion.h"
#include "tf2_ros/transform_listener.h"
#include "tf2_ros/buffer.h"
#include "moveit/move_group_interface/move_group_interface.h"
#include "moveit/planning_scene_interface/planning_scene_interface.h"
#include "moveit_msgs/msg/constraints.hpp"
#include "moveit_msgs/msg/joint_constraint.hpp"
#include "interfaces/srv/arm_pose.hpp"

constexpr float TCP_HEIGHT = 0.063;

//Function to generate a collision object
auto generateCollisionObject(float sx,float sy, float sz, float x, float y, float z, std::string frame_id, std::string id) {
    moveit_msgs::msg::CollisionObject collision_object;
    collision_object.header.frame_id = frame_id;
    collision_object.id = id;
    shape_msgs::msg::SolidPrimitive primitive;

    primitive.type = primitive.BOX;
    primitive.dimensions.resize(3);
    primitive.dimensions[primitive.BOX_X] = sx;
    primitive.dimensions[primitive.BOX_Y] = sy;
    primitive.dimensions[primitive.BOX_Z] = sz;

    geometry_msgs::msg::Pose box_pose;
    box_pose.orientation.w = 1.0; 
    box_pose.position.x = x;
    box_pose.position.y = y;
    box_pose.position.z = z;

    collision_object.primitives.push_back(primitive);
    collision_object.primitive_poses.push_back(box_pose);
    collision_object.operation = collision_object.ADD;

    return collision_object;
}

//Function to generate a target position message
auto generatePoseMsg(float x,float y, float z,float qx,float qy,float qz,float qw) {
    geometry_msgs::msg::Pose msg;
    msg.orientation.x = qx;
    msg.orientation.y = qy;
    msg.orientation.z = qz;
    msg.orientation.w = qw;
    msg.position.x = x;
    msg.position.y = y;
    msg.position.z = z;
    return msg;
}


class MoveToLocation : public rclcpp::Node
{
    public:
        MoveToLocation() : Node("move_to_location")
        {
            // Create a marker subscriber
            blockMarkerSub_ = this->create_subscription<visualization_msgs::msg::Marker>("block_markers", 10, std::bind(&MoveToLocation::markerSubCallback, this, std::placeholders::_1));

            // Create arm movement service
            armPoseService_ = create_service<interfaces::srv::ArmPose>("arm_pose", std::bind(&MoveToLocation::moveArm, this, std::placeholders::_1, std::placeholders::_2));
        
            // Generate the movegroup interface
            moveGroupInterface_ = std::make_unique<moveit::planning_interface::MoveGroupInterface>(std::shared_ptr<rclcpp::Node>(this), "ur_manipulator");
            moveGroupInterface_->setPlanningTime(3.0);
            moveGroupInterface_->setNumPlanningAttempts(10);
            moveGroupInterface_->setMaxVelocityScalingFactor(0.08);
            moveGroupInterface_->setPlannerId("RRTstarkConfigDefault");

            // Joint Constraints
            moveit_msgs::msg::Constraints constraints;
            moveit_msgs::msg::JointConstraint jointConstraint;

            jointConstraint.joint_name = "shoulder_pan_joint";  
            jointConstraint.position = 0.0;        
            jointConstraint.tolerance_above = M_PI / 2.0;   
            jointConstraint.tolerance_below = M_PI / 2.0; 
            jointConstraint.weight = 1.0;
            constraints.joint_constraints.push_back(jointConstraint);

            jointConstraint.joint_name = "elbow_joint";  
            jointConstraint.position = M_PI / 2.0;        
            jointConstraint.tolerance_above = M_PI / 2.0;   
            jointConstraint.tolerance_below = M_PI / 2.0; 
            jointConstraint.weight = 1.0;  
            constraints.joint_constraints.push_back(jointConstraint);

            jointConstraint.joint_name = "wrist_1_joint";  
            jointConstraint.position = -M_PI / 2.0;        
            jointConstraint.tolerance_above = M_PI / 2.0;   
            jointConstraint.tolerance_below = M_PI / 2.0; 
            jointConstraint.weight = 1.0;  
            constraints.joint_constraints.push_back(jointConstraint);

            jointConstraint.joint_name = "wrist_2_joint";  
            jointConstraint.position = -M_PI / 2.0;        
            jointConstraint.tolerance_above = M_PI / 2.0;   
            jointConstraint.tolerance_below = M_PI / 2.0; 
            jointConstraint.weight = 1.0;  
            constraints.joint_constraints.push_back(jointConstraint);

            jointConstraint.joint_name = "wrist_3_joint";  
            jointConstraint.position = 0;        
            jointConstraint.tolerance_above = M_PI;   
            jointConstraint.tolerance_below = M_PI; 
            jointConstraint.weight = 1.0;  
            constraints.joint_constraints.push_back(jointConstraint);
            
            moveGroupInterface_->setPathConstraints(constraints);

            std::string frame_id = moveGroupInterface_->getPlanningFrame();

            // Generate the objects to avoid
            auto colObjectBackwall = generateCollisionObject( 2.4, 0.04, 1.0, 0.85, -0.3, 0.5, frame_id, "backWall");
            auto colObjectSidewall = generateCollisionObject( 0.04, 1.2, 1.0, -0.3, 0.25, 0.5, frame_id, "sideWall");
            auto colObjectTable = generateCollisionObject(2.4, 1.2, 0.04, 0.85, 0.25, 0.05, frame_id, "floor");

            moveit::planning_interface::PlanningSceneInterface planningSceneInterface;
            
            // Apply collision objects
            planningSceneInterface.applyCollisionObject(colObjectBackwall);
            planningSceneInterface.applyCollisionObject(colObjectSidewall);
            planningSceneInterface.applyCollisionObject(colObjectTable);

            // Move to home position
            tf2::Quaternion q;
            q.setRPY(M_PI, 0.0, -M_PI / 2.0);
            auto const home = generatePoseMsg(0.0, 0.5, 0.2 + TCP_HEIGHT, q.x(), q.y(), q.z(), q.w());
            moveGroupInterface_->setPoseTarget(home);
            moveit::planning_interface::MoveGroupInterface::Plan planMessage;
            bool success = static_cast<bool>(moveGroupInterface_->plan(planMessage));
            // Execute movement
            if (success) {
                moveGroupInterface_->execute(planMessage);
            }
        }

    private:

        void moveArm(const std::shared_ptr<interfaces::srv::ArmPose::Request> request, std::shared_ptr<interfaces::srv::ArmPose::Response> response)
        {
            tf2::Quaternion q;
            q.setRPY(M_PI, 0.0, -M_PI / 2.0 + request->yaw);
            auto const targetPose = generatePoseMsg(request->x, request->y, request->z + TCP_HEIGHT, q.x(), q.y(), q.z(), q.w());

            // Check if moving to the same pose
            if (previousPose_ == targetPose) {
                response->success = true;
                RCLCPP_INFO(get_logger(), "Same pose called. No movements.");
                return;
            }
            previousPose_ = targetPose;

            // Plan movement
            moveGroupInterface_->setPoseTarget(targetPose);
            moveit::planning_interface::MoveGroupInterface::Plan planMessage;
            bool success = static_cast<bool>(moveGroupInterface_->plan(planMessage));
            // Execute movement
            if (success) {
                moveGroupInterface_->execute(planMessage);
                response->success = true;
            } else {
                response->success = false;
            }
        }

        void markerSubCallback(const visualization_msgs::msg::Marker::SharedPtr markerMsg) {
            float sx = markerMsg->scale.x;
            float sy = markerMsg->scale.y;
            float sz = markerMsg->scale.z;
            float x = markerMsg->pose.position.x;
            float y = markerMsg->pose.position.y;
            float z = markerMsg->pose.position.z;
            std::string frame_id = moveGroupInterface_->getPlanningFrame();
            std::string name = "block " + std::to_string(markerMsg->id);
            auto markerObject = generateCollisionObject(sx, sy, sz, x, y, z, frame_id, name);

            moveit::planning_interface::PlanningSceneInterface planningSceneInterface;
            planningSceneInterface.applyCollisionObject(markerObject);
        }


    std::shared_ptr<moveit::planning_interface::MoveGroupInterface> moveGroupInterface_;
    rclcpp::Subscription<visualization_msgs::msg::Marker>::SharedPtr blockMarkerSub_;
    rclcpp::Service<interfaces::srv::ArmPose>::SharedPtr armPoseService_;
    geometry_msgs::msg::Pose previousPose_;
};


int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<MoveToLocation>());
    rclcpp::shutdown();
    return 0;
}
