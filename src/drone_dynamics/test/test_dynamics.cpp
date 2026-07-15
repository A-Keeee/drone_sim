#include <gtest/gtest.h>
#include "ake_drone_sim/dynamics_model.hpp"
using namespace ake_drone_sim;
TEST(Dynamics, EqualMotorsProduceNoTorque) {
  DynamicsModel model; model.setMotorCommand(Eigen::Vector4d::Constant(5000));
  for(int i=0;i<500;++i) model.step(.005);
  EXPECT_NEAR(model.wrench().tail<3>().norm(),0.0,1e-8);
}
TEST(Dynamics, QuaternionRemainsNormalized) {
  DynamicsModel model;VehicleState state;state.body_rate={.3,-.2,.4};model.reset(state);
  for(int i=0;i<1000;++i) model.step(.002);
  EXPECT_NEAR(model.state().orientation.norm(),1.0,1e-9);
}
