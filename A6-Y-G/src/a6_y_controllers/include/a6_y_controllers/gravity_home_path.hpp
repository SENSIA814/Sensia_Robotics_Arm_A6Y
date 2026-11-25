#pragma once
#include <algorithm>
#include <array>
#include <cmath>
#include <cstdint>
#include <vector>

namespace a6_y_controllers
{
struct GravityHomePath
{
  std::vector<double> times, positions, velocities;
  uint64_t revision{0};
  bool cancel{false};

  void sample(double t, std::array<double, 6>& q, std::array<double, 6>& v) const
  {
    if (t >= times.back()) {
      for (int j=0;j<6;++j) {q[j]=positions[positions.size()-6+j];v[j]=0;}
      return;
    }
    const auto found=std::upper_bound(times.begin(),times.end(),t);
    const size_t i=found==times.begin()?0:size_t(found-times.begin()-1);
    const double h=times[i+1]-times[i], x=std::clamp((t-times[i])/h,0.,1.);
    for(int j=0;j<6;++j) {
      const double a=positions[6*i+j], b=positions[6*(i+1)+j];
      const double u=velocities[6*i+j], w=velocities[6*(i+1)+j];
      q[j]=(2*x*x*x-3*x*x+1)*a+(x*x*x-2*x*x+x)*h*u+(-2*x*x*x+3*x*x)*b+(x*x*x-x*x)*h*w;
      v[j]=(6*x*x-6*x)*(a-b)/h+(3*x*x-4*x+1)*u+(3*x*x-2*x)*w;
    }
  }

  bool valid(const std::array<double,6>& lower, const std::array<double,6>& upper) const
  {
    if(times.size()<2 || times.size()>10000 || positions.size()!=6*times.size() ||
       velocities.size()!=positions.size() || times.front()!=0. || times.back()>120.) return false;
    for(double x:times) if(!std::isfinite(x)) return false;
    for(double x:positions) if(!std::isfinite(x)) return false;
    for(double x:velocities) if(!std::isfinite(x)) return false;
    for(int j=0;j<6;++j)
      if(std::abs(velocities[j])>1e-6 || std::abs(velocities[velocities.size()-6+j])>1e-6 ||
         std::abs(positions[positions.size()-6+j])>1e-4) return false;
    // Exact cubic extrema for bounds, velocity and acceleration. Slow return:
    // <=0.15 rad/s, <=0.3 rad/s² independent of the planner's settings.
    for(size_t i=0;i+1<times.size();++i) {
      const double h=times[i+1]-times[i];
      if(h<.001) return false;
      for(int j=0;j<6;++j) {
        const double p=positions[6*i+j], u=velocities[6*i+j];
        const double end=positions[6*(i+1)+j], w=velocities[6*(i+1)+j];
        const double a=(2*p-2*end+h*(u+w))/(h*h*h);
        const double b=(-3*p+3*end-h*(2*u+w))/(h*h);
        const auto check=[&](double t) {
          const double q=((a*t+b)*t+u)*t+p, v=(3*a*t+2*b)*t+u;
          return q>=lower[j]-1e-6 && q<=upper[j]+1e-6 && std::abs(v)<=.150001 && std::abs(6*a*t+2*b)<=.300001;
        };
        if(!check(0) || !check(h)) return false;
        if(std::abs(a)>1e-12) {
          const double vertex=-b/(3*a);
          if(vertex>0 && vertex<h && !check(vertex)) return false;
          const double d=4*b*b-12*a*u;
          if(d>=0) for(double t:{(-2*b+std::sqrt(d))/(6*a),(-2*b-std::sqrt(d))/(6*a)})
            if(t>0 && t<h && !check(t)) return false;
        } else if(std::abs(b)>1e-12) {
          const double t=-u/(2*b); if(t>0 && t<h && !check(t)) return false;
        }
      }
    }
    return true;
  }
};
}
