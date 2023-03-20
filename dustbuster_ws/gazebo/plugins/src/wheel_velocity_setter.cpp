/**
 *  Copyright (c) 2023, Onur Ulusoy
 *  All rights reserved.
 *  This code is licensed under the MIT License.
 * 
 * @file wheel_velocity_setter.cpp
 * @brief Plugin for setting the speed of wheels in Gazebo simulation.
 * 
 * This plugin allows the other scripts to set the speed of wheels in a Gazebo simulation, 
 * which can be useful for testing and development of autonomous vehicle algorithms 
 * or other robotics applications. It provides an interface for setting the speed 
 * of each wheel individually, and can be used in conjunction with other plugins 
 * or software to create more complex control systems.
 * 
 * This plugin requires the use of the Gazebo simulator, and is designed to be used 
 * with models that have wheels. It is compatible with both linear and angular velocity 
 * controllers, and can be customized to work with different types of wheels and vehicles.
 * 
 * Usage: To use this plugin, simply add it to the Gazebo model that you wish to control, 
 * and publish to necessary ros2 topics (/left_joint_velocity, /right_joint_velocity) 
 * the angular velocity in rad/s. Joint names should be left_joint_velocity and
 * right_joint_velocity in gazebo .sdf file.
 * 
 * 
 * @author Onur Ulusoy
 * @date 17.03.2023
 * 
 */


#include <gazebo/common/Plugin.hh>
#include <gazebo/physics/physics.hh>

#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/float64.hpp"

namespace gazebo {
  class VelocitySetterPlugin : public ModelPlugin {

  public:
    VelocitySetterPlugin() {}

    void Load(physics::ModelPtr model, sdf::ElementPtr sdf) {
      // Initialize ROS 2
      //rclcpp::init(0, nullptr);

      // Create a node for the publisher
      this->node = rclcpp::Node::make_shared("my_publisher_node");

      // Create publishers for the joint velocities
      this->leftJointPublisher = node->create_publisher<std_msgs::msg::Float64>("left_joint_velocity", 10);
      this->rightJointPublisher = node->create_publisher<std_msgs::msg::Float64>("right_joint_velocity", 10);

      this->subLeftJoint = node->create_subscription<std_msgs::msg::Float64>(
      "left_joint_velocity", 10, std::bind(&VelocitySetterPlugin::leftJointTopicCallback, this, std::placeholders::_1));

      this->subRightJoint = node->create_subscription<std_msgs::msg::Float64>(
      "right_joint_velocity", 10, std::bind(&VelocitySetterPlugin::rightJointTopicCallback, this, std::placeholders::_1));

      // Get the left and right joints
      this->leftJoint = model->GetJoint("left_wheel_joint");
      this->rightJoint = model->GetJoint("right_wheel_joint");

      // Connect the update event callback
      this->updateConnection = event::Events::ConnectWorldUpdateBegin(
        std::bind(&VelocitySetterPlugin::OnUpdate, this));
    }

    void OnUpdate() {
      rclcpp::spin_some(this->node);
    }

    void leftJointTopicCallback(const std_msgs::msg::Float64::SharedPtr msg) const
    {
        // Set the right wheel joint velocity in rad/s acquired from the topic
        double leftWheelVelocity = msg->data;
        this->leftJoint->SetVelocity(0, leftWheelVelocity);
    }

    void rightJointTopicCallback(const std_msgs::msg::Float64::SharedPtr msg) const
    {
        // Set the right wheel joint velocity in rad/s acquired from the topic
        double rightWheelVelocity = msg->data;
        this->rightJoint->SetVelocity(0, rightWheelVelocity);
    }

  private:

    rclcpp::Node::SharedPtr node;
    
    rclcpp::Publisher<std_msgs::msg::Float64>::SharedPtr leftJointPublisher;
    rclcpp::Publisher<std_msgs::msg::Float64>::SharedPtr rightJointPublisher;

    rclcpp::Subscription<std_msgs::msg::Float64>::SharedPtr subLeftJoint;
    rclcpp::Subscription<std_msgs::msg::Float64>::SharedPtr subRightJoint;

    physics::JointPtr leftJoint;
    physics::JointPtr rightJoint;

    event::ConnectionPtr updateConnection;

  };

  GZ_REGISTER_MODEL_PLUGIN(VelocitySetterPlugin)
}
