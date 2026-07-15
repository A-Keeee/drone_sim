#include "ake_drone_sim/voxel_grid.hpp"
#include "ake_drone_sim/pointcloud_utils.hpp"
#include <rclcpp/rclcpp.hpp>
#include <visualization_msgs/msg/marker_array.hpp>

using namespace std::chrono_literals;
namespace ake_drone_sim {
class VoxelMapNode final:public rclcpp::Node{
 public:VoxelMapNode():Node("voxel_map_node"){
  const double res=declare_parameter("resolution",0.2);auto origin=declare_parameter<std::vector<double>>("origin",{-2.,-4.,0.});auto size=declare_parameter<std::vector<double>>("size",{12.,8.,5.});grid_.configure({origin[0],origin[1],origin[2]},{size[0],size[1],size[2]},res);
  boxes_=declare_parameter<std::vector<double>>("obstacles",{1.5,0.,1.2,0.6,2.0,2.4, 3.0,-1.2,2.0,0.8,1.6,4.0, 4.0,1.2,2.0,0.8,1.6,4.0, 5.4,0.,3.7,0.8,3.5,1.0, 6.8,0.,1.3,0.8,4.5,2.6});
  for(size_t i=0;i+5<boxes_.size();i+=6)grid_.addBox({boxes_[i],boxes_[i+1],boxes_[i+2]},{boxes_[i+3],boxes_[i+4],boxes_[i+5]});
  auto qos=rclcpp::QoS(1).transient_local().reliable();cloud_pub_=create_publisher<sensor_msgs::msg::PointCloud2>("/map/obstacles",qos);marker_pub_=create_publisher<visualization_msgs::msg::MarkerArray>("/map/obstacle_markers",qos);timer_=create_wall_timer(1s,std::bind(&VoxelMapNode::publish,this));publish();}
 private:void publish(){std_msgs::msg::Header h;h.stamp=now();h.frame_id="map";cloud_pub_->publish(makeCloud(h,grid_.occupiedPoints()));visualization_msgs::msg::MarkerArray a;for(size_t i=0,id=0;i+5<boxes_.size();i+=6,++id){visualization_msgs::msg::Marker m;m.header=h;m.ns="obstacles";m.id=id;m.type=m.CUBE;m.action=m.ADD;m.pose.position=pointMsg({boxes_[i],boxes_[i+1],boxes_[i+2]});m.pose.orientation.w=1;m.scale=vectorMsg({boxes_[i+3],boxes_[i+4],boxes_[i+5]});m.color.r=.75;m.color.g=.2;m.color.b=.12;m.color.a=.65;a.markers.push_back(m);}marker_pub_->publish(a);}
 VoxelGrid grid_;std::vector<double>boxes_;rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr cloud_pub_;rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr marker_pub_;rclcpp::TimerBase::SharedPtr timer_;
};}
int main(int argc,char**argv){rclcpp::init(argc,argv);rclcpp::spin(std::make_shared<ake_drone_sim::VoxelMapNode>());rclcpp::shutdown();}
