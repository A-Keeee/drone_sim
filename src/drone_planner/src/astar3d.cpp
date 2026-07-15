#include "ake_drone_sim/astar3d.hpp"
#include <queue>
#include <unordered_map>
#include <algorithm>

namespace ake_drone_sim {
namespace {
struct Record { Eigen::Vector3i idx; double g{1e100}; int64_t parent{-1}; bool closed{false}; };
struct Open { double f; int64_t key; bool operator>(const Open &o) const { return f > o.f; } };
int64_t keyOf(const Eigen::Vector3i &i, const Eigen::Vector3i &d) {
  return (static_cast<int64_t>(i.x()) * d.y() + i.y()) * d.z() + i.z();
}
}
AStar3D::Result AStar3D::plan(const VoxelGrid &grid, const Eigen::Vector3d &start,
                             const Eigen::Vector3d &goal) const {
  Result result; const auto si=grid.worldToGrid(start), gi=grid.worldToGrid(goal), dims=grid.dimensions();
  if (!grid.inside(si)||!grid.inside(gi)) { result.reason="start_or_goal_outside_map"; return result; }
  if (grid.occupied(si)||grid.occupied(gi)) { result.reason="start_or_goal_occupied"; return result; }
  std::unordered_map<int64_t,Record> rec; rec.reserve(std::min(max_nodes_,static_cast<size_t>(200000)));
  std::priority_queue<Open,std::vector<Open>,std::greater<Open>> open;
  const int64_t sk=keyOf(si,dims), gk=keyOf(gi,dims); rec[sk]={si,0.0,-1,false};
  open.push({weight_*(gi-si).cast<double>().norm(),sk});
  while(!open.empty() && result.expanded<max_nodes_) {
    auto cur=open.top(); open.pop(); auto it=rec.find(cur.key); if(it==rec.end()||it->second.closed) continue;
    Record &node=it->second; node.closed=true; ++result.expanded;
    if(cur.key==gk) {
      int64_t k=gk; while(k>=0) { const auto &n=rec.at(k); result.path.push_back(grid.gridToWorld(n.idx)); k=n.parent; }
      std::reverse(result.path.begin(),result.path.end()); result.path.front()=start; result.path.back()=goal;
      result.path=simplify(grid,result.path); result.success=true; result.reason="ok"; return result;
    }
    for(int dx=-1;dx<=1;++dx) for(int dy=-1;dy<=1;++dy) for(int dz=-1;dz<=1;++dz) {
      if(dx==0&&dy==0&&dz==0) continue;
      Eigen::Vector3i ni=node.idx+Eigen::Vector3i(dx,dy,dz);
      if(!grid.inside(ni)||grid.occupied(ni)) continue;
      if(!grid.segmentFree(grid.gridToWorld(node.idx),grid.gridToWorld(ni))) continue;
      const int64_t nk=keyOf(ni,dims); const double ng=node.g+Eigen::Vector3d(dx,dy,dz).norm();
      auto [nit,inserted]=rec.emplace(nk,Record{ni});
      if(!nit->second.closed && ng<nit->second.g) { nit->second.g=ng; nit->second.parent=cur.key;
        open.push({ng+weight_*(gi-ni).cast<double>().norm(),nk}); }
    }
  }
  result.reason = result.expanded>=max_nodes_ ? "search_budget_exhausted" : "no_path"; return result;
}
std::vector<Eigen::Vector3d> AStar3D::simplify(const VoxelGrid &grid,const std::vector<Eigen::Vector3d>&p){
  if(p.size()<3)return p;
  std::vector<Eigen::Vector3d> out{p.front()}; size_t i=0;
  while(i+1<p.size()){size_t j=p.size()-1;while(j>i+1&&!grid.segmentFree(p[i],p[j]))--j;out.push_back(p[j]);i=j;}return out;
}
}  // namespace ake_drone_sim
