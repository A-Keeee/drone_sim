#include <gtest/gtest.h>
#include "ake_drone_sim/astar3d.hpp"
#include "ake_drone_sim/bspline.hpp"
using namespace ake_drone_sim;
TEST(AStar3D, FindsVerticalDetour) {
  VoxelGrid grid({0,0,0},{6,4,4},.25);grid.addBox({3,2,1.2},{.5,4,2.4});
  auto inflated=grid.inflated(.25);auto result=AStar3D().plan(inflated,{.5,2,.5},{5.5,2,.5});
  ASSERT_TRUE(result.success)<<result.reason;double max_z=0;for(const auto&p:result.path)max_z=std::max(max_z,p.z());EXPECT_GT(max_z,2.5);
}
TEST(PlanningPipeline, FiveObstacleSplineIsSafe) {
  VoxelGrid grid({-2,-4,0},{12,8,5},.2);
  grid.addBox({1.5,0,1.2},{.6,2,2.4});grid.addBox({3,-1.2,2},{.8,1.6,4});grid.addBox({4,1.2,2},{.8,1.6,4});grid.addBox({5.4,0,3.7},{.8,3.5,1});grid.addBox({6.8,0,1.3},{.8,4.5,2.6});
  auto inflated=grid.inflated(.68);auto result=AStar3D().plan(inflated,{0,0,1.5},{8,0,1.5});ASSERT_TRUE(result.success)<<result.reason;
  BSpline spline;spline.setControlPoints(makeSplineControlPoints(result.path,.1),.25);ASSERT_TRUE(spline.valid());
  for(double u=0;u<=spline.maxU();u+=.02)EXPECT_FALSE(inflated.occupiedWorld(spline.position(u)));
}
