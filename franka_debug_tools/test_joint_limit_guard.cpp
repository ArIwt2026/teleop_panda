#include "joint_limit_guard.hpp"
#include <cassert>
#include <iostream>
int main() {
  JointLimitGuard g;
  g.lower.fill(-1); g.upper.fill(1); g.stiffness.fill(60);
  g.damping.fill(8); g.torque_cap.fill(10);
  g.activation=.17; g.reserve=.05; g.prediction=.1;
  g.validate();
  for(size_t i=0;i<7;++i) {
    assert(g.torque(i,0,0)==0);
    assert(g.torque(i,.9,0)<0);
    assert(g.torque(i,-.9,0)>0);
    assert(g.torque(i,.9,.2)<g.torque(i,.9,-.2));
    assert(g.torque(i,-.9,-.2)>g.torque(i,-.9,.2));
    assert(std::abs(g.torque(i,.99,10))<=10);
    assert(g.inside(i,.96));
    assert(!g.inside(i,1.0));
    assert(!g.inside(i,-1.0));
    assert(g.inside(i,-.96));
    // Cross the inner reserve and retreat: resistance must aid retreat,
    // remain finite and capped, and smoothly vanish outside the wall.
    for(double q=.99;q>.7;q-=.0001) {
      assert(g.inside(i,q));
      assert(g.torque(i,q,-.1)<=0);
      assert(g.torque(i,-q,.1)>=0);
      assert(std::isfinite(g.torque(i,q,-.1)));
      assert(std::abs(g.torque(i,q,-.1))<=10);
    }
    assert(std::abs(g.torque(i,.95000001,0)-g.torque(i,.94999999,0))<1e-4);
    assert(g.inside(i,.9));
    assert(std::abs(g.torque(i,.83000001,0))<1e-10);
    for(double q=-.99;q<1;q+=.01)
      assert(std::abs(g.torque(i,q,.2)+g.torque(i,-q,-.2))<1e-10);
  }
  g.reserve=g.activation;
  bool rejected=false;
  try {g.validate();} catch(const std::invalid_argument&) {rejected=true;}
  assert(rejected);
  std::cout << "Joint limit guard checks passed\n";
}
