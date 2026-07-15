#include "ake_drone_sim/voxel_grid.hpp"
#include "ake_drone_sim/pointcloud_utils.hpp"
#include <rclcpp/rclcpp.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <random>

using namespace std::chrono_literals;
namespace ake_drone_sim {
class LidarNode final:public rclcpp::Node{
 public:LidarNode():Node("simulated_lidar_node"),rng_(declare_parameter<int>("random_seed",17)){
  resolution_=declare_parameter("map_resolution",0.2);grid_.configure({-2,-4,0},{12,8,5},resolution_);hfov_=declare_parameter("horizontal_fov_deg",120.)*M_PI/180.;vfov_=declare_parameter("vertical_fov_deg",60.)*M_PI/180.;hbeams_=declare_parameter("horizontal_beams",121);vbeams_=declare_parameter("vertical_beams",31);min_range_=declare_parameter("min_range",0.2);max_range_=declare_parameter("max_range",8.0);noise_=declare_parameter("range_noise",0.01);dropout_=declare_parameter("dropout",0.0);
  auto qos=rclcpp::QoS(1).transient_local().reliable();map_sub_=create_subscription<sensor_msgs::msg::PointCloud2>("/map/obstacles",qos,[this](sensor_msgs::msg::PointCloud2::SharedPtr m){grid_.clear();for(const auto&p:readCloud(*m))grid_.set(grid_.worldToGrid(p));have_map_=true;});
  odom_sub_=create_subscription<nav_msgs::msg::Odometry>("/drone/ground_truth/odom",10,[this](nav_msgs::msg::Odometry::SharedPtr m){position_=eigen(m->pose.pose.position);orientation_=eigen(m->pose.pose.orientation).normalized();have_pose_=true;});pub_=create_publisher<sensor_msgs::msg::PointCloud2>("/drone/lidar/points",rclcpp::SensorDataQoS());timer_=create_wall_timer(100ms,std::bind(&LidarNode::scan,this));}
 private:void scan(){if(!have_map_||!have_pose_)return;std::vector<Eigen::Vector3d>hits;std::normal_distribution<double>n(0,noise_);std::uniform_real_distribution<double>u(0,1);Eigen::Vector3d sensor=position_+orientation_*Eigen::Vector3d(.08,0,0);
  for(int vi=0;vi<vbeams_;++vi){double el=-vfov_/2+vfov_*vi/std::max(1,vbeams_-1);for(int hi=0;hi<hbeams_;++hi){if(u(rng_)<dropout_)continue;double az=-hfov_/2+hfov_*hi/std::max(1,hbeams_-1);Eigen::Vector3d local(std::cos(el)*std::cos(az),std::cos(el)*std::sin(az),std::sin(el));Eigen::Vector3d dir=orientation_*local;double hit=0;
    for(double r=min_range_;r<=max_range_;r+=0.45*resolution_){
      if(grid_.occupiedWorld(sensor+r*dir)){hit=std::max(min_range_,r+n(rng_));break;}
    }
    if(hit>0) hits.push_back(local*hit+Eigen::Vector3d(.08,0,0));}}
  std_msgs::msg::Header h;h.stamp=now();h.frame_id="lidar_link";pub_->publish(makeCloud(h,hits));}
 VoxelGrid grid_;double resolution_,hfov_,vfov_,min_range_,max_range_,noise_,dropout_;int hbeams_,vbeams_;bool have_map_{false},have_pose_{false};Eigen::Vector3d position_;Eigen::Quaterniond orientation_;std::mt19937 rng_;rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr map_sub_;rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_sub_;rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr pub_;rclcpp::TimerBase::SharedPtr timer_;
};}
int main(int argc,char**argv){rclcpp::init(argc,argv);rclcpp::spin(std::make_shared<ake_drone_sim::LidarNode>());rclcpp::shutdown();}
