#include <a6_y_controllers/A6_y_gravity_comp_controller.hpp>
#include <a6_y_controllers/gravity_home_brake.hpp>
#include <pinocchio/algorithm/crba.hpp>
#include <stdexcept>
#include <sstream>
#include <iomanip>

namespace a6_y_controllers
{
void A6YGravityCompController::reset_follow()
{
  follow_requested_=FollowCommand{};follow_buffer_.reset();
  follow_rt_revision_.store(0);follow_phase_.store(0);follow_fault_.store(0);
  follow_fault_joint_.store(-1);follow_fault_value_.store(0.);follow_fault_limit_.store(0.);
  follow_q_.setZero();follow_v_.setZero();follow_integral_.setZero();follow_elapsed_=0;
}

void A6YGravityCompController::accept_follow(const FollowService::Request& r, FollowService::Response& out)
{
  out.success=false;
  if(r.command==0) out.success=true; // Status never refreshes the reference watchdog.
  else if(!home_controller_active_.load() || !enable_output_) out.message="Controller is not active";
  else if(home_pending_.load() || home_phase_.load()==1 || home_phase_.load()==2) out.message="Home must be cancelled/acknowledged first";
  else if(r.command>3 || (r.command!=2 && r.revision!=follow_requested_.revision)) out.message="Invalid command or stale revision";
  else if(r.command==1 && (follow_phase_.load()!=2 || follow_rt_revision_.load()!=follow_requested_.revision)) out.message="Start requires acknowledged hold";
  else if(r.command==3 && follow_phase_.load()==1) out.message="Hold before returning to gravity";
  else {
    FollowCommand c;c.command=r.command==3?0:r.command;
    c.revision=++follow_next_revision_;c.received=staged_output_steady_time_seconds();
    follow_requested_=c;follow_buffer_.writeFromNonRT(c);out.success=true;
  }
  out.revision=follow_requested_.revision;out.applied_revision=follow_rt_revision_.load();
  out.phase=follow_phase_.load();
  if(out.message.empty()) {
    switch(follow_fault_.load()) {
      case 1: out.message="Reference timeout; hold latched";break;
      case 2: out.message="Tracking error exceeds 0.15 rad; hold latched";break;
      case 3: {
        std::ostringstream message;message<<std::fixed<<std::setprecision(6);
        message<<"Joint speed outside motor VMAX feedback range; hold latched joint="
               <<joint_names_[follow_fault_joint_.load()]<<" value="<<follow_fault_value_.load()
               <<" limit="<<follow_fault_limit_.load()<<" rad/s";
        out.message=message.str();break;
      }
      case 4: out.message="Measured joint outside limits; hold latched";break;
      default: out.message="OK";
    }
  }
}

void A6YGravityCompController::accept_follow_reference(const FollowReference& r)
{
  // Depth one, ordered revisions and original timestamps prevent replay/backlog
  // from authorizing movement. A late packet cannot clear a latched hold.
  if(follow_requested_.command!=1 || follow_phase_.load()!=1 ||
     r.revision!=follow_requested_.revision || r.sequence<=follow_requested_.sequence) return;
  if(r.stamp_ns<=0) return;
  const double age=(get_node()->now().nanoseconds()-r.stamp_ns)*1e-9;
  if(!std::isfinite(age) || age<0 || age>.15) return;
  for(int i=0;i<6;++i)
    if(!std::isfinite(r.positions[i]) || r.positions[i]<home_lower_[i] || r.positions[i]>home_upper_[i]) return;
  follow_requested_.sequence=r.sequence;follow_requested_.positions=r.positions;
  follow_requested_.received=staged_output_steady_time_seconds()-age;
  follow_buffer_.writeFromNonRT(follow_requested_);
}

bool A6YGravityCompController::follow_effort(double dt, double now, const Eigen::VectorXd& gravity, JointVector& effort)
{
  if(!follow_enabled_) return false;
  const auto* c=follow_buffer_.readFromRT();
  JointVector q,v;
  for(int i=0;i<6;++i) {q[i]=q_[pinocchio_q_indices_[i]];v[i]=dq_[pinocchio_v_indices_[i]];}
  if(c->revision && c->revision!=follow_rt_revision_.load()) {
    const bool was_holding=follow_phase_.load()!=0;
    follow_q_=q;follow_v_.setZero();follow_integral_.setZero();
    follow_elapsed_=was_holding?2.:0.;follow_fault_.store(0);
    follow_phase_.store(c->command);follow_rt_revision_.store(c->revision);
  }
  if(follow_phase_.load()==0) return false;
  if(!std::isfinite(dt) || dt<=0 || dt>.1) throw std::runtime_error("Invalid follow control period");
  int fault=0;
  if(follow_phase_.load()==1 && (now<c->received || now-c->received>.25)) fault=1;
  for(int i=0;i<6 && !fault;++i) {
    if(std::abs(follow_q_[i]-q[i])>.15) fault=2;
    else if(std::abs(v[i])>feedback_velocity_limits_[i]) {
      fault=3;
      // Preserve the first snapshot after a hold has latched.
      if(follow_phase_.load()!=3) {
        follow_fault_joint_.store(i);follow_fault_value_.store(v[i]);
        follow_fault_limit_.store(feedback_velocity_limits_[i]);
      }
    }
    else if(q[i]<home_lower_[i]-.01 || q[i]>home_upper_[i]+.01) fault=4;
  }
  if(fault && follow_phase_.load()!=3) {
    follow_fault_.store(fault);follow_phase_.store(3);
    follow_q_=q;follow_v_.setZero();follow_integral_.setZero();
  }
  if(follow_phase_.load()==1 && c->sequence) {
    for(int i=0;i<6;++i) {
      // Bounded reference speed/acceleration; no queued trajectories to chase.
      const double desired=std::clamp(2.*(c->positions[i]-follow_q_[i]),-.10,.10);
      follow_v_[i]+=std::clamp(desired-follow_v_[i],-.20*dt,.20*dt);
      const double next=follow_q_[i]+follow_v_[i]*dt;
      follow_q_[i]=std::clamp(next,home_lower_[i],home_upper_[i]);
      if(next!=follow_q_[i]) follow_v_[i]=0.;
    }
  } else follow_v_.setZero();
  if(teaching_position_output_) {effort.setZero();return true;}
  JointVector error=follow_q_-q,relative=v-follow_v_,lower,upper,base;
  const JointVector stiffness((JointVector()<<54.,50.,54.,22.5,38.5,5.2).finished());
  const JointVector damping((JointVector()<<6.,4.5,3.6,1.8,1.8,.52).finished());
  const JointVector budget((JointVector()<<2.4,2.8,3.6,.6,.6,.6).finished());
  const JointVector ki((JointVector()<<4.,4.,8.,.8,1.2,1.).finished());
  const JointVector imax((JointVector()<<1.5,2.2,3.,.4,.4,.35).finished());
  for(int i=0;i<6;++i) {
    base[i]=std::clamp(effort_signs_[i]*torque_scales_[i]*gravity[i],-torque_limits_[i],torque_limits_[i]);
    lower[i]=std::max(-budget[i],-torque_limits_[i]-base[i]);upper[i]=std::min(budget[i],torque_limits_[i]-base[i]);
    if(std::abs(error[i])>.003 && error[i]*follow_integral_[i]<0) follow_integral_[i]=0;
    follow_integral_[i]=std::clamp(follow_integral_[i],lower[i],upper[i]);
  }
  pinocchio::crba(*pinocchio_model_,*pinocchio_data_,q_);
  pinocchio_data_->M.template triangularView<Eigen::StrictlyLower>()=pinocchio_data_->M.transpose().template triangularView<Eigen::StrictlyLower>();
  JointMatrix mass;
  for(int i=0;i<6;++i) for(int j=0;j<6;++j) mass(i,j)=pinocchio_data_->M(pinocchio_v_indices_[i],pinocchio_v_indices_[j]);
  const double ramp=std::min(1.,follow_elapsed_/2.);
  const auto step=bounded_joint_impedance(mass,error,relative,ramp*stiffness,damping,dt,lower-follow_integral_,upper-follow_integral_);
  if(!step.valid) throw std::runtime_error("Invalid follow impedance solution");
  for(int i=0;i<6;++i) {
    const auto brake=i==3?gravity_home_brake(follow_integral_[i]+step.torque[i],v[i],lower[i],upper[i]):
      GravityHomeBrakeResult{true,false,follow_integral_[i]+step.torque[i]};
    if(!brake.valid) throw std::runtime_error("Invalid follow brake");
    if(brake.braking && follow_integral_[i]*v[i]>0) follow_integral_[i]=0;
    effort[i]=std::clamp(base[i]+brake.torque,-torque_limits_[i],torque_limits_[i]);
    if(follow_phase_.load()!=3 && !brake.braking && ramp>=1 && std::abs(error[i])>.003 && std::abs(v[i])<=.15 &&
       !(step.saturated[i] && error[i]*(follow_integral_[i]+step.torque[i])>=0))
      follow_integral_[i]=std::clamp(follow_integral_[i]+dt*ki[i]*error[i],-imax[i],imax[i]);
  }
  follow_elapsed_+=dt;
  return true;
}
}
