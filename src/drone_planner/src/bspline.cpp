#include "ake_drone_sim/bspline.hpp"
#include <algorithm>

namespace ake_drone_sim {
void BSpline::setControlPoints(const std::vector<Eigen::Vector3d>&p,double dt){control_=p;dt_=std::max(1e-3,dt);}
namespace { void segment(const std::vector<Eigen::Vector3d>&c,double maxu,double u,int &k,double&t){
  u=clamp(u,0.0,maxu-1e-10);k=std::min(static_cast<int>(std::floor(u)),static_cast<int>(c.size())-4);t=u-k;}}
Eigen::Vector3d BSpline::position(double u)const{if(!valid())return Eigen::Vector3d::Zero();int k;double t;segment(control_,maxU(),u,k,t);
  const double t2=t*t,t3=t2*t;return ((1-3*t+3*t2-t3)*control_[k]+(4-6*t2+3*t3)*control_[k+1]+(1+3*t+3*t2-3*t3)*control_[k+2]+t3*control_[k+3])/6.0;}
Eigen::Vector3d BSpline::velocity(double u)const{if(!valid())return Eigen::Vector3d::Zero();int k;double t;segment(control_,maxU(),u,k,t);
  const double t2=t*t;return ((-3+6*t-3*t2)*control_[k]+(-12*t+9*t2)*control_[k+1]+(3+6*t-9*t2)*control_[k+2]+3*t2*control_[k+3])/(6.0*dt_);}
Eigen::Vector3d BSpline::acceleration(double u)const{if(!valid())return Eigen::Vector3d::Zero();int k;double t;segment(control_,maxU(),u,k,t);
  return ((6-6*t)*control_[k]+(-12+18*t)*control_[k+1]+(6-18*t)*control_[k+2]+6*t*control_[k+3])/(6.0*dt_*dt_);}
void ArcLengthTable::build(const BSpline&s,int n){us_.clear();lengths_.clear();if(!s.valid())return;n=std::max(2,n);Eigen::Vector3d prev=s.position(0);double l=0;
  for(int i=0;i<n;++i){double u=s.maxU()*i/(n-1.0);auto p=s.position(u);if(i)l+=(p-prev).norm();us_.push_back(u);lengths_.push_back(l);prev=p;}}
double ArcLengthTable::uFromLength(double l)const{if(us_.empty())return 0;l=clamp(l,0.0,lengths_.back());auto it=std::lower_bound(lengths_.begin(),lengths_.end(),l);if(it==lengths_.begin())return us_.front();size_t i=it-lengths_.begin();double a=(l-lengths_[i-1])/std::max(1e-9,lengths_[i]-lengths_[i-1]);return us_[i-1]+a*(us_[i]-us_[i-1]);}
double ArcLengthTable::lengthFromU(double u)const{if(us_.empty())return 0;u=clamp(u,us_.front(),us_.back());auto it=std::lower_bound(us_.begin(),us_.end(),u);if(it==us_.begin())return lengths_.front();size_t i=it-us_.begin();double a=(u-us_[i-1])/std::max(1e-9,us_[i]-us_[i-1]);return lengths_[i-1]+a*(lengths_[i]-lengths_[i-1]);}
double ArcLengthTable::closestU(const BSpline&s,const Eigen::Vector3d&p,double guess)const{if(us_.empty())return 0;double best=clamp(guess,0.0,s.maxU()),d=(s.position(best)-p).squaredNorm();
  for(double u:us_){double nd=(s.position(u)-p).squaredNorm();if(nd<d){d=nd;best=u;}}double step=us_.size()>1?us_[1]-us_[0]:0.01;for(int i=0;i<8;++i){double left=clamp(best-step,0.0,s.maxU()),right=clamp(best+step,0.0,s.maxU());double dl=(s.position(left)-p).squaredNorm(),dr=(s.position(right)-p).squaredNorm();if(dl<d){best=left;d=dl;}if(dr<d){best=right;d=dr;}step*=0.5;}return best;}
std::vector<Eigen::Vector3d> makeSplineControlPoints(const std::vector<Eigen::Vector3d>&path,double spacing){if(path.empty())return{};std::vector<Eigen::Vector3d> sampled{path.front()};double carry=0;
  for(size_t i=1;i<path.size();++i){auto a=path[i-1],b=path[i];double len=(b-a).norm();for(double d=spacing-carry;d<len;d+=spacing)sampled.push_back(a+(b-a)*(d/len));carry=std::fmod(carry+len,spacing);}if((sampled.back()-path.back()).norm()>1e-6)sampled.push_back(path.back());
  std::vector<Eigen::Vector3d> c;c.push_back(sampled.front());c.push_back(sampled.front());c.insert(c.end(),sampled.begin(),sampled.end());c.push_back(sampled.back());c.push_back(sampled.back());return c;}
}  // namespace ake_drone_sim
