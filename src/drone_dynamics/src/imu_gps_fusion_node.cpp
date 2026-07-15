#include "ake_drone_sim/eskf.hpp"
#include "ake_drone_sim/ros_utils.hpp"
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/imu.hpp>
#include <geometry_msgs/msg/pose_with_covariance_stamped.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <nav_msgs/msg/path.hpp>
#include <tf2_ros/transform_broadcaster.h>
#include <tf2_ros/static_transform_broadcaster.h>

namespace ake_drone_sim {
class FusionNode final:public rclcpp::Node{
 public:FusionNode():Node("imu_gps_fusion_node"){
  imu_sub_=create_subscription<sensor_msgs::msg::Imu>("/drone/imu",rclcpp::SensorDataQoS(),std::bind(&FusionNode::imu,this,std::placeholders::_1));
  gps_sub_=create_subscription<geometry_msgs::msg::PoseWithCovarianceStamped>("/drone/gps_pose",10,std::bind(&FusionNode::gps,this,std::placeholders::_1));
  odom_pub_=create_publisher<nav_msgs::msg::Odometry>("/drone/odom",10);path_pub_=create_publisher<nav_msgs::msg::Path>("/drone/path",10);path_.header.frame_id="map";
  tf_=std::make_unique<tf2_ros::TransformBroadcaster>(*this);static_tf_=std::make_unique<tf2_ros::StaticTransformBroadcaster>(*this);
  geometry_msgs::msg::TransformStamped st;st.header.stamp=now();st.header.frame_id="base_link";st.child_frame_id="lidar_link";st.transform.translation.x=0.08;st.transform.rotation.w=1.0;static_tf_->sendTransform(st);
 }
 private:void gps(const geometry_msgs::msg::PoseWithCovarianceStamped::SharedPtr m){auto q=eigen(m->pose.pose.orientation);if(!filter_.initialized())filter_.initialize(eigen(m->pose.pose.position),q,rclcpp::Time(m->header.stamp).seconds());else filter_.updatePose(eigen(m->pose.pose.position),q);}
 void imu(const sensor_msgs::msg::Imu::SharedPtr m){if(!filter_.initialized())return;filter_.predict(eigen(m->angular_velocity),eigen(m->linear_acceleration),rclcpp::Time(m->header.stamp).seconds());filter_.observeImuOrientation(eigen(m->orientation));publish(m->header.stamp);}
 void publish(const builtin_interfaces::msg::Time&s){nav_msgs::msg::Odometry o;o.header.stamp=s;o.header.frame_id="map";o.child_frame_id="base_link";o.pose.pose.position=pointMsg(filter_.position());o.pose.pose.orientation=quatMsg(filter_.orientation());o.twist.twist.linear=vectorMsg(filter_.velocity());o.twist.twist.angular=vectorMsg(filter_.bodyRate());const auto&p=filter_.covariance();for(int i=0;i<3;++i){o.pose.covariance[i*6+i]=p(i,i);o.twist.covariance[i*6+i]=p(i+3,i+3);}odom_pub_->publish(o);
  geometry_msgs::msg::TransformStamped t;t.header=o.header;t.child_frame_id="base_link";t.transform.translation=vectorMsg(filter_.position());t.transform.rotation=o.pose.pose.orientation;tf_->sendTransform(t);
  if(++div_>=10){div_=0;geometry_msgs::msg::PoseStamped ps;ps.header=o.header;ps.pose=o.pose.pose;path_.header.stamp=s;path_.poses.push_back(ps);if(path_.poses.size()>5000)path_.poses.erase(path_.poses.begin(),path_.poses.begin()+1000);path_pub_->publish(path_);}}
 Eskf filter_;int div_{0};nav_msgs::msg::Path path_;rclcpp::Subscription<sensor_msgs::msg::Imu>::SharedPtr imu_sub_;rclcpp::Subscription<geometry_msgs::msg::PoseWithCovarianceStamped>::SharedPtr gps_sub_;rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr odom_pub_;rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr path_pub_;std::unique_ptr<tf2_ros::TransformBroadcaster>tf_;std::unique_ptr<tf2_ros::StaticTransformBroadcaster>static_tf_;
};}
int main(int argc,char**argv){rclcpp::init(argc,argv);rclcpp::spin(std::make_shared<ake_drone_sim::FusionNode>());rclcpp::shutdown();}
