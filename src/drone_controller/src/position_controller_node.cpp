#include "ake_drone_sim/se3_controller.hpp"
#include "ake_drone_sim/ros_utils.hpp"
#include "drone_msgs/msg/trajectory_setpoint.hpp"
#include <rclcpp/rclcpp.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <std_msgs/msg/float32_multi_array.hpp>

using namespace std::chrono_literals;
namespace ake_drone_sim {
class ControllerNode final:public rclcpp::Node{
 public:ControllerNode():Node("position_controller_node"){
  odom_sub_=create_subscription<nav_msgs::msg::Odometry>("/drone/odom",10,[this](nav_msgs::msg::Odometry::SharedPtr m){state_.position=eigen(m->pose.pose.position);state_.velocity=eigen(m->twist.twist.linear);state_.orientation=eigen(m->pose.pose.orientation).normalized();state_.body_rate=eigen(m->twist.twist.angular);last_odom_=now();have_odom_=true;});
  ref_sub_=create_subscription<drone_msgs::msg::TrajectorySetpoint>("/drone/reference",10,[this](drone_msgs::msg::TrajectorySetpoint::SharedPtr m){reference_.position=eigen(m->position);reference_.velocity=eigen(m->velocity);reference_.acceleration=eigen(m->acceleration);reference_.yaw=m->yaw;reference_.yaw_rate=m->yaw_rate;last_ref_=now();have_ref_=true;});
  pub_=create_publisher<std_msgs::msg::Float32MultiArray>("/drone/motor_rpm_cmd",10);timer_=create_wall_timer(10ms,std::bind(&ControllerNode::tick,this));}
 private:void tick(){Eigen::Vector4d rpm=Eigen::Vector4d::Zero();if(have_odom_&&have_ref_)rpm=controller_.compute(state_,reference_);std_msgs::msg::Float32MultiArray m;for(int i=0;i<4;++i)m.data.push_back(static_cast<float>(rpm[i]));pub_->publish(m);}
 Se3Controller controller_;ControlState state_;ControlReference reference_;bool have_odom_{false},have_ref_{false};rclcpp::Time last_odom_{0,0,RCL_ROS_TIME},last_ref_{0,0,RCL_ROS_TIME};rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_sub_;rclcpp::Subscription<drone_msgs::msg::TrajectorySetpoint>::SharedPtr ref_sub_;rclcpp::Publisher<std_msgs::msg::Float32MultiArray>::SharedPtr pub_;rclcpp::TimerBase::SharedPtr timer_;
};}
int main(int argc,char**argv){rclcpp::init(argc,argv);rclcpp::spin(std::make_shared<ake_drone_sim::ControllerNode>());rclcpp::shutdown();}
