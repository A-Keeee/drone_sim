#include "ake_drone_sim/dynamics_model.hpp"
#include "ake_drone_sim/ros_utils.hpp"
#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/float32_multi_array.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <nav_msgs/msg/path.hpp>
#include <sensor_msgs/msg/imu.hpp>
#include <geometry_msgs/msg/pose_with_covariance_stamped.hpp>
#include <tf2_ros/transform_broadcaster.h>
#include <random>

using namespace std::chrono_literals;
namespace ake_drone_sim {
class DynamicsNode final : public rclcpp::Node {
 public:
  DynamicsNode():Node("quadrotor_dynamics_node"),rng_(declare_parameter<int>("random_seed",7)){
    VehicleParams p; p.mass=declare_parameter("mass",1.0);p.arm_length=declare_parameter("arm_length",0.17);
    p.inertia={declare_parameter("inertia_x",0.008),declare_parameter("inertia_y",0.008),declare_parameter("inertia_z",0.014)};
    p.thrust_coefficient=declare_parameter("thrust_coefficient",8.54858e-6);p.torque_coefficient=declare_parameter("torque_coefficient",1.6e-7);
    p.motor_time_constant=declare_parameter("motor_time_constant",0.035);p.max_rpm=declare_parameter("max_rpm",9000.0);model_=std::make_unique<DynamicsModel>(p);
    imu_noise_=declare_parameter("imu_accel_noise",0.025);gyro_noise_=declare_parameter("imu_gyro_noise",0.002);gps_noise_=declare_parameter("gps_position_noise",0.04);
    cmd_sub_=create_subscription<std_msgs::msg::Float32MultiArray>("/drone/motor_rpm_cmd",10,[this](std_msgs::msg::Float32MultiArray::SharedPtr m){Eigen::Vector4d r=Eigen::Vector4d::Zero();if(m->data.size()==4)for(int i=0;i<4;++i)r[i]=m->data[i];model_->setMotorCommand(r);last_cmd_=now();});
    odom_pub_=create_publisher<nav_msgs::msg::Odometry>("/drone/ground_truth/odom",10);imu_pub_=create_publisher<sensor_msgs::msg::Imu>("/drone/imu",rclcpp::SensorDataQoS());
    gps_pub_=create_publisher<geometry_msgs::msg::PoseWithCovarianceStamped>("/drone/gps_pose",10);rpm_pub_=create_publisher<std_msgs::msg::Float32MultiArray>("/drone/motor_rpm",10);
    path_pub_=create_publisher<nav_msgs::msg::Path>("/drone/ground_truth/path",10);tf_=std::make_unique<tf2_ros::TransformBroadcaster>(*this);path_.header.frame_id="map";
    timer_=create_wall_timer(5ms,std::bind(&DynamicsNode::tick,this));
  }
 private:
  void tick(){const auto stamp=now();if(last_stamp_.nanoseconds()==0){last_stamp_=stamp;return;}double dt=(stamp-last_stamp_).seconds();last_stamp_=stamp;if((stamp-last_cmd_).seconds()>0.3)model_->setMotorCommand(Eigen::Vector4d::Zero());model_->step(std::min(dt,0.01));const auto&s=model_->state();
    nav_msgs::msg::Odometry o;o.header.stamp=stamp;o.header.frame_id="map";o.child_frame_id="ground_truth/base_link";o.pose.pose.position=pointMsg(s.position);o.pose.pose.orientation=quatMsg(s.orientation);o.twist.twist.linear=vectorMsg(s.velocity);o.twist.twist.angular=vectorMsg(s.body_rate);odom_pub_->publish(o);
    sensor_msgs::msg::Imu imu;imu.header=o.header;imu.header.frame_id="base_link";imu.orientation=o.pose.pose.orientation;std::normal_distribution<double>n(0,1);Eigen::Vector3d sf=s.orientation.inverse()*(s.world_acceleration+kGravity*Eigen::Vector3d::UnitZ());for(int i=0;i<3;++i){sf[i]+=imu_noise_*n(rng_);}imu.linear_acceleration=vectorMsg(sf);Eigen::Vector3d gyro=s.body_rate;for(int i=0;i<3;++i)gyro[i]+=gyro_noise_*n(rng_);imu.angular_velocity=vectorMsg(gyro);imu_pub_->publish(imu);
    std_msgs::msg::Float32MultiArray rpm;for(int i=0;i<4;++i)rpm.data.push_back(static_cast<float>(s.motor_rpm[i]));rpm_pub_->publish(rpm);
    geometry_msgs::msg::TransformStamped t;t.header=o.header;t.child_frame_id="ground_truth/base_link";t.transform.translation=vectorMsg(s.position);t.transform.rotation=o.pose.pose.orientation;tf_->sendTransform(t);
    if(++path_div_>=10){path_div_=0;geometry_msgs::msg::PoseStamped ps;ps.header=o.header;ps.pose=o.pose.pose;path_.header.stamp=stamp;path_.poses.push_back(ps);if(path_.poses.size()>5000)path_.poses.erase(path_.poses.begin(),path_.poses.begin()+1000);path_pub_->publish(path_);}
    if(++gps_div_>=20){gps_div_=0;geometry_msgs::msg::PoseWithCovarianceStamped g;g.header=o.header;Eigen::Vector3d p=s.position;for(int i=0;i<3;++i)p[i]+=gps_noise_*n(rng_);g.pose.pose.position=pointMsg(p);Eigen::Vector3d angle(0.01*n(rng_),0.01*n(rng_),0.01*n(rng_));g.pose.pose.orientation=quatMsg(s.orientation*Eigen::Quaterniond(Eigen::AngleAxisd(angle.norm(),angle.norm()>1e-9?angle.normalized():Eigen::Vector3d::UnitX())));for(int i=0;i<3;++i)g.pose.covariance[i*6+i]=gps_noise_*gps_noise_;gps_pub_->publish(g);}
  }
  std::unique_ptr<DynamicsModel>model_;std::mt19937 rng_;double imu_noise_,gyro_noise_,gps_noise_;rclcpp::Time last_stamp_{0,0,RCL_ROS_TIME},last_cmd_{0,0,RCL_ROS_TIME};int gps_div_{0},path_div_{0};nav_msgs::msg::Path path_;
  rclcpp::Subscription<std_msgs::msg::Float32MultiArray>::SharedPtr cmd_sub_;rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr odom_pub_;rclcpp::Publisher<sensor_msgs::msg::Imu>::SharedPtr imu_pub_;rclcpp::Publisher<geometry_msgs::msg::PoseWithCovarianceStamped>::SharedPtr gps_pub_;rclcpp::Publisher<std_msgs::msg::Float32MultiArray>::SharedPtr rpm_pub_;rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr path_pub_;std::unique_ptr<tf2_ros::TransformBroadcaster>tf_;rclcpp::TimerBase::SharedPtr timer_;
};
}
int main(int argc,char**argv){rclcpp::init(argc,argv);rclcpp::spin(std::make_shared<ake_drone_sim::DynamicsNode>());rclcpp::shutdown();}
