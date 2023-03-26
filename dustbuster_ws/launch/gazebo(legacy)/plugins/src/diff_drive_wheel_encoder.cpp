#include <gazebo/gazebo.hh>
#include <gazebo/physics/physics.hh>
#include <gazebo/common/common.hh>
#include <rclcpp/rclcpp.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2_ros/transform_broadcaster.h>
#include <geometry_msgs/msg/transform_stamped.hpp>

namespace gazebo
{
  class DiffDriveWheelEncoder : public ModelPlugin
  {
    public: void Load(physics::ModelPtr _parent, sdf::ElementPtr _sdf)
    {
      // Store the model pointer
      this->model = _parent;
      
      // Create ROS node
      this->node = std::make_shared<rclcpp::Node>("diff_drive_wheel_encoder_plugin");

      // Create odometry publisher
      this->odom_pub = this->node->create_publisher<nav_msgs::msg::Odometry>("odom", 1);

      // Initialize update connection
      this->updateConnection = event::Events::ConnectWorldUpdateBegin(
          std::bind(&DiffDriveWheelEncoder::OnUpdate, this));

      // Initialize odometry message
      this->odom.header.frame_id = "odom";
      this->odom.child_frame_id = "chassis";
    }

    public: void OnUpdate()
    {
      // Get current time
      rclcpp::Time current_time = this->node->now();

      // Get model pose
      ignition::math::Pose3d pose = this->model->WorldPose();

      // Update odometry message
      this->odom.header.stamp = current_time;
      this->odom.pose.pose.position.x = pose.Pos().X();
      this->odom.pose.pose.position.y = pose.Pos().Y();
      this->odom.pose.pose.position.z = pose.Pos().Z();
      this->odom.pose.pose.orientation.x = pose.Rot().X();
      this->odom.pose.pose.orientation.y = pose.Rot().Y();
      this->odom.pose.pose.orientation.z = pose.Rot().Z();
      this->odom.pose.pose.orientation.w = pose.Rot().W();

      // Publish odometry message
      this->odom_pub->publish(this->odom);

      // Publish the transform
      static tf2_ros::TransformBroadcaster br(this->node);
      geometry_msgs::msg::TransformStamped transform;
      transform.header.stamp = current_time;
      transform.header.frame_id = "odom";
      transform.child_frame_id = "chassis";
      transform.transform.translation.x = pose.Pos().X();
      transform.transform.translation.y = pose.Pos().Y();
      transform.transform.translation.z = pose.Pos().Z();
      transform.transform.rotation.x = pose.Rot().X();
      transform.transform.rotation.y = pose.Rot().Y();
      transform.transform.rotation.z = pose.Rot().Z();
      transform.transform.rotation.w = pose.Rot().W();
      br.sendTransform(transform);
    }

    private: physics::ModelPtr model;
    private: event::ConnectionPtr updateConnection;
    private: rclcpp::Node::SharedPtr node;
    private: rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr odom_pub;
    private: nav_msgs::msg::Odometry odom;
  };

  // Register this plugin with the simulator
  GZ_REGISTER_MODEL_PLUGIN(DiffDriveWheelEncoder)
}
