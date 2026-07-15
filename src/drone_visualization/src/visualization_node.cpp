#include "ake_drone_sim/ros_utils.hpp"
#include <rclcpp/rclcpp.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <visualization_msgs/msg/marker_array.hpp>
namespace ake_drone_sim {
class VisualizationNode final:public rclcpp::Node{
 public:VisualizationNode():Node("visualization_node"){
  pub_=create_publisher<visualization_msgs::msg::MarkerArray>("/drone/markers",10);odom_=create_subscription<nav_msgs::msg::Odometry>("/drone/odom",10,[this](nav_msgs::msg::Odometry::SharedPtr m){pose_=m->pose.pose;publish();});goal_=create_subscription<geometry_msgs::msg::PoseStamped>("/drone/goal",10,[this](geometry_msgs::msg::PoseStamped::SharedPtr m){goal_pose_=m->pose;have_goal_=true;publish();});safe_=create_subscription<geometry_msgs::msg::PoseStamped>("/drone/safe_goal",10,[this](geometry_msgs::msg::PoseStamped::SharedPtr m){safe_pose_=m->pose;have_safe_=true;publish();});}
 private:void publish(){visualization_msgs::msg::MarkerArray a;std_msgs::msg::Header h;h.stamp=now();h.frame_id="map";visualization_msgs::msg::Marker d;d.header=h;d.ns="drone";d.id=0;d.type=d.CUBE;d.action=d.ADD;d.pose=pose_;d.scale=vectorMsg({.35,.35,.08});d.color.r=.1;d.color.g=.45;d.color.b=1;d.color.a=1;a.markers.push_back(d);if(have_goal_)a.markers.push_back(sphere(h,"goal",1,goal_pose_,.22,0.1,1,0.1));if(have_safe_)a.markers.push_back(sphere(h,"safe_goal",2,safe_pose_,.14,1,1,0));pub_->publish(a);}
 visualization_msgs::msg::Marker sphere(const std_msgs::msg::Header&h,const std::string&ns,int id,const geometry_msgs::msg::Pose&p,double scale,float r,float g,float b){visualization_msgs::msg::Marker m;m.header=h;m.ns=ns;m.id=id;m.type=m.SPHERE;m.action=m.ADD;m.pose=p;m.scale=vectorMsg({scale,scale,scale});m.color.r=r;m.color.g=g;m.color.b=b;m.color.a=1;return m;}
 geometry_msgs::msg::Pose pose_,goal_pose_,safe_pose_;bool have_goal_{false},have_safe_{false};rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr pub_;rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_;rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr goal_,safe_;
};}
int main(int argc,char**argv){rclcpp::init(argc,argv);rclcpp::spin(std::make_shared<ake_drone_sim::VisualizationNode>());rclcpp::shutdown();}
