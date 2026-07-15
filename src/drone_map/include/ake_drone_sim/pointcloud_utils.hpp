#pragma once
#include "ake_drone_sim/ros_utils.hpp"
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <sensor_msgs/point_cloud2_iterator.hpp>
#include <std_msgs/msg/header.hpp>
#include <vector>
namespace ake_drone_sim {
inline sensor_msgs::msg::PointCloud2 makeCloud(const std_msgs::msg::Header&h,const std::vector<Eigen::Vector3d>&points){sensor_msgs::msg::PointCloud2 c;c.header=h;sensor_msgs::PointCloud2Modifier mod(c);mod.setPointCloud2FieldsByString(1,"xyz");mod.resize(points.size());sensor_msgs::PointCloud2Iterator<float>x(c,"x"),y(c,"y"),z(c,"z");for(const auto&p:points){*x=p.x();*y=p.y();*z=p.z();++x;++y;++z;}return c;}
inline std::vector<Eigen::Vector3d> readCloud(const sensor_msgs::msg::PointCloud2&c){std::vector<Eigen::Vector3d>p;p.reserve(c.width*c.height);sensor_msgs::PointCloud2ConstIterator<float>x(c,"x"),y(c,"y"),z(c,"z");for(;x!=x.end();++x,++y,++z)if(std::isfinite(*x)&&std::isfinite(*y)&&std::isfinite(*z))p.emplace_back(*x,*y,*z);return p;}
}
