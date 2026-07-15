#pragma once
#include "ake_drone_sim/common.hpp"
#include <geometry_msgs/msg/point.hpp>
#include <geometry_msgs/msg/quaternion.hpp>
#include <geometry_msgs/msg/vector3.hpp>
namespace ake_drone_sim {
inline Eigen::Vector3d eigen(const geometry_msgs::msg::Point&p){return{p.x,p.y,p.z};}
inline Eigen::Vector3d eigen(const geometry_msgs::msg::Vector3&p){return{p.x,p.y,p.z};}
inline Eigen::Quaterniond eigen(const geometry_msgs::msg::Quaternion&q){return{q.w,q.x,q.y,q.z};}
inline geometry_msgs::msg::Point pointMsg(const Eigen::Vector3d&p){geometry_msgs::msg::Point m;m.x=p.x();m.y=p.y();m.z=p.z();return m;}
inline geometry_msgs::msg::Vector3 vectorMsg(const Eigen::Vector3d&p){geometry_msgs::msg::Vector3 m;m.x=p.x();m.y=p.y();m.z=p.z();return m;}
inline geometry_msgs::msg::Quaternion quatMsg(const Eigen::Quaterniond&q){geometry_msgs::msg::Quaternion m;m.w=q.w();m.x=q.x();m.y=q.y();m.z=q.z();return m;}
}
