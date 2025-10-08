#include <memory>
#include <functional>

#include "rclcpp/rclcpp.hpp"
#include "tf2/exceptions.h"

#include "tf2/LinearMath/Quaternion.h"
#include "tf2_ros/transform_listener.h"
#include "tf2_ros/buffer.h"

#include "std_msgs/msg/bool.hpp"
#include "std_msgs/msg/string.hpp"

#include "interfaces/msg/start_end_points.hpp"
#include "interfaces/msg/part_size.hpp"
#include "interfaces/srv/get_part_size.hpp"

#include "visualization_msgs/msg/marker.hpp"

using std::placeholders::_1;
#define pass (void)0

class Brain :public rclcpp::Node{
  public:
    Brain() : Node("brain")
    {
        // Create a timer 
        timer_ = this->create_wall_timer(std::chrono::milliseconds(1000), std::bind(&Brain::timerCallback, this));
        // Create a publisher for motion requests
        startEndPointPub_ = this->create_publisher<interfaces::msg::StartEndPoints>("start_end_points", 10);
        // Create a subscriber
        motionStateSub_ = this->create_subscription<std_msgs::msg::Bool>("motion_state", 10, std::bind(&Brain::subscriberCallback, this, _1));

        // Create a TF listener
        tf_buffer_ =   std::make_unique<tf2_ros::Buffer>(this->get_clock());
        tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);

        blockMarkerPub_ = this->create_publisher<visualization_msgs::msg::Marker>("block_markers", 10);

        stlSlicerCli_ = this->create_client<interfaces::srv::GetPartSize>("get_part_size");
        while (!stlSlicerCli_->wait_for_service(std::chrono::seconds(1))) {
          if (!rclcpp::ok()) {
            RCLCPP_ERROR(rclcpp::get_logger("rclcpp"), "Interrupted while waiting for the service. Exiting.");
          }
          RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "Slicer service not available, waiting again...");
        }

        auto request = std::make_shared<interfaces::srv::GetPartSize::Request>();
        request->file_path = "/home/mtrn/4231/group-assignment-brick-layers/src/stl_parser/stl_files/mvp.stl";

        // auto result = stlSlicerCli_->async_send_request(request);
        auto future_result = stlSlicerCli_->async_send_request(request, std::bind(&Brain::got_part_size, this, std::placeholders::_1));
        RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "Running service , %d", 1);
        
        //Initialized frame names
        std::string cam_tf = "camera";
        std::string bl_tf = "base_link";
        std::string block_tf = "free_block";

        // Initialize variables
        current_block = 0;
        previous_block = -1;
        new_block_  = false;
        isRobotMoving_ = false;
        stopTimer_ = false;
        sizeX = 0;
        sizeY = 0;
        sizeZ = 0;
        recievedPart =false;

      }

  private:

    void timerCallback() {
      if (stopTimer_) return;
      executeLoop();
    }

    void executeLoop() {

      if (isRobotMoving_ || !recievedPart) return;

      // Publish previous block as a marker
      if (previous_block != -1) {
        publish_block_marker(std::to_string(previous_block), previous_block, previous_block_coords[0], previous_block_coords[1], previous_block_coords[2]);
      }

      std::string toFrameRelStart = free_block_search();
      std::string toFrameRelEnd = "target_block" + std::to_string(current_block);
      std::string fromFrameRel = "base_link";

      interfaces::msg::StartEndPoints msg;
      geometry_msgs::msg::TransformStamped start;
      geometry_msgs::msg::TransformStamped end;

      // Look up for the transformations
      try {
        end = tf_buffer_->lookupTransform(fromFrameRel, toFrameRelEnd, tf2::TimePointZero);
      } 
      catch (const tf2::TransformException & ex) {
        RCLCPP_INFO(
          this->get_logger(), "Could not transform %s to %s: %s",
          toFrameRelEnd.c_str(), fromFrameRel.c_str(), ex.what());
        stopTimer_ = true;
        RCLCPP_INFO(this->get_logger(), "Program Finished!");
        return;
      }
      try {
        start = tf_buffer_->lookupTransform(fromFrameRel, toFrameRelStart, tf2::TimePointZero);
      } 
      catch (const tf2::TransformException & ex) {
        RCLCPP_INFO(
          this->get_logger(), "Could not transform %s to %s: %s",
          toFrameRelStart.c_str(), fromFrameRel.c_str(), ex.what());
        return;
      }
      // RCLCPP_INFO(this->get_logger(), "Free block location %f, %f, %f", start.transform.translation.x, start.transform.translation.y, start.transform.translation.z);

      msg.start_x = start.transform.translation.x;
      msg.start_y = start.transform.translation.y;
      msg.start_z = start.transform.translation.z;
      msg.start_yaw = 0.0;
      msg.end_x = end.transform.translation.x;
      msg.end_y = end.transform.translation.y;
      msg.end_z = end.transform.translation.z;
      msg.end_yaw = 0.0;

      startEndPointPub_->publish(msg);
      RCLCPP_INFO(this->get_logger(), "Moving to block %ld in orientation %f", current_block, board_frame_orientation);
      previous_block = current_block;
      previous_block_coords[0] = msg.end_x;
      previous_block_coords[1] = msg.end_y;
      previous_block_coords[2] = msg.end_z;
      current_block++;

    }

    std::string free_block_search() {
      bool looking_for_brick = true;
      int block_cand = 0;
      geometry_msgs::msg::TransformStamped block_loc;
      geometry_msgs::msg::TransformStamped marker_1_loc;
      geometry_msgs::msg::TransformStamped marker_2_loc;
      std::string brick_id = "";

      while(looking_for_brick){
        try {
          brick_id = "free_block_" + std::to_string(block_cand);
          block_loc = tf_buffer_->lookupTransform(
            brick_id, "board_frame",
            tf2::TimePointZero);
          RCLCPP_INFO(this->get_logger(), "Free block coords with respect to board frame: %f, %f", block_loc.transform.translation.x, block_loc.transform.translation.y);
          if(!(block_loc.transform.translation.x > 0 && block_loc.transform.translation.x < 0.4 && block_loc.transform.translation.y > 0 && block_loc.transform.translation.y < 0.3)){
            return brick_id;
          }
          marker_1_loc = tf_buffer_->lookupTransform(brick_id, "marker_1", tf2::TimePointZero);
          marker_2_loc = tf_buffer_->lookupTransform(brick_id, "marker_2", tf2::TimePointZero);
          board_frame_orientation = atan2(marker_2_loc.transform.translation.y - marker_1_loc.transform.translation.y, marker_2_loc.transform.translation.x - marker_1_loc.transform.translation.x);

          block_cand++;
          
        } 
        catch (const tf2::TransformException & ex) {
          RCLCPP_INFO(
            this->get_logger(), "Could not transform %s to %s: %s",
            brick_id.c_str(), "marker_n", ex.what());
          looking_for_brick = false;
          return "";
        }
      }
      return "";
    }

    void subscriberCallback(const std_msgs::msg::Bool::SharedPtr motionStateMsg) {
      isRobotMoving_ = motionStateMsg->data;
    }

    void got_part_size(rclcpp::Client<interfaces::srv::GetPartSize>::SharedFuture future) {

      if (future.get()) {
        sizeX = future.get()->size_x;
        sizeY = future.get()->size_y;
        sizeZ = future.get()->size_z;
        RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "Loaded part with size: %d, %d, %d", sizeX, sizeY, sizeZ);
      } else {
        RCLCPP_ERROR(get_logger(), "Service call failed");
      }
      recievedPart = true;

    }


  void publish_block_marker(std::string name, int id, float x,float y,float z) {
    
    visualization_msgs::msg::Marker block_marker;

    block_marker.header.frame_id = "base_link";
    block_marker.ns = name;
    block_marker.id = id;
    block_marker.type = 1;
    block_marker.action = 0;
    block_marker.pose.position.x = x;
    block_marker.pose.position.y = y;
    block_marker.pose.position.z = z;
    block_marker.pose.orientation.x = 0.0;
    block_marker.pose.orientation.y = 0.0;
    block_marker.pose.orientation.z = 0.0;
    block_marker.pose.orientation.w = 1.0;
    block_marker.scale.x = 0.02;
    block_marker.scale.y = 0.02;
    block_marker.scale.z = 0.02;
    block_marker.color.a = 1.0; // Don't forget to set the alpha!
    block_marker.color.r = 0.0;
    block_marker.color.g = 1.0;
    block_marker.color.b = 0.0;
    //only if using a MESH_RESOURCE marker type:
    block_marker.mesh_resource = "package://pr2_description/meshes/base_v0/base.dae";
    blockMarkerPub_->publish( block_marker );
  }
    
    bool isRobotMoving_;
    bool stopTimer_;
    bool new_block_;
    size_t current_block;
    int previous_block;
    float previous_block_coords[3];
    float board_frame_orientation;

    int sizeX;
    int sizeY;
    int sizeZ;
    bool recievedPart;

    rclcpp::TimerBase::SharedPtr timer_;
    rclcpp::Publisher<interfaces::msg::StartEndPoints>::SharedPtr startEndPointPub_;
    rclcpp::Publisher<visualization_msgs::msg::Marker>::SharedPtr blockMarkerPub_;
    rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr motionStateSub_;
    rclcpp::Subscription<interfaces::msg::PartSize>::SharedPtr partSizeSub_;
    rclcpp::Client<interfaces::srv::GetPartSize>::SharedPtr stlSlicerCli_; 
    std::shared_ptr<tf2_ros::TransformListener> tf_listener_{nullptr};
    std::unique_ptr<tf2_ros::Buffer> tf_buffer_;
    std::string cam_tf;
    std::string bl_tf;
    std::string block_tf;
};


int main(int argc, char ** argv)
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<Brain>());
    rclcpp::shutdown();
    return 0;
}
