#include "ake_drone_sim/mpc_smoother.hpp"
namespace ake_drone_sim {
std::vector<KinematicState>MpcSmoother::solve(const KinematicState&initial,const std::vector<KinematicState>&ref)const{
  std::vector<KinematicState>out;out.reserve(config_.horizon);KinematicState x=initial;
  for(int k=0;k<config_.horizon;++k){const auto&r=ref[std::min(k,static_cast<int>(ref.size())-1)];Eigen::Vector3d desired=config_.kp*(r.p-x.p)+config_.kv*(r.v-x.v)+r.a;
    if(desired.norm()>config_.max_accel) desired=desired.normalized()*config_.max_accel;
    Eigen::Vector3d jerk=(desired-x.a)/config_.dt;
    if(jerk.norm()>config_.max_jerk) jerk=jerk.normalized()*config_.max_jerk;
    x.p+=x.v*config_.dt+0.5*x.a*config_.dt*config_.dt+jerk*config_.dt*config_.dt*config_.dt/6.0;x.v+=x.a*config_.dt+0.5*jerk*config_.dt*config_.dt;x.a+=jerk*config_.dt;
    if(x.v.norm()>config_.max_velocity) x.v=x.v.normalized()*config_.max_velocity;
    out.push_back(x);}return out;}
}  // namespace ake_drone_sim
