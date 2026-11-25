#include <a6_y_ros2_control/accepted_position.hpp>
#include <cmath>
#include <iostream>
#include <limits>

int main() {
  using a6_y_ros2_control::AcceptedPosition;
  for (double scale : {1.0, 0.0239}) {
    AcceptedPosition a; double q=0;
    if (!a.update(.1*scale,1,.2*scale,.4,q) || q!=.1*scale) return 1;
    for (int i=1;i<=4;++i) {
      if (a.update(.5*scale,1+.02*i,.2*scale,.4,q) || a.valid || a.consecutive_rejections!=unsigned(i) ||
          std::abs(a.age(1+.02*i)-.02*i)>1e-12 || q!=.1*scale) return 2;
    }
    if (!a.update(.11*scale,1.1,.2*scale,.4,q) || a.consecutive_rejections || !a.valid || a.age(1.1)!=0) return 3;
    if (a.update(std::numeric_limits<double>::quiet_NaN(),1.12,.2*scale,.4,q) || !std::isfinite(q)) return 4;
  }
  std::cout<<"Accepted position age, rad/m thresholds, rejection recovery and NaN checks passed\n";
}
