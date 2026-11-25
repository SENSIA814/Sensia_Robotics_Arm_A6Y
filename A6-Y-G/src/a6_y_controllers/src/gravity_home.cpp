#include <a6_y_controllers/A6_y_gravity_comp_controller.hpp>
#include <a6_y_controllers/gravity_home_brake.hpp>
#include <pinocchio/algorithm/crba.hpp>
#include <iomanip>
#include <sstream>

namespace a6_y_controllers
{
void A6YGravityCompController::clear_home_fault()
{
  home_fault_.store(HomeFault::None);home_fault_revision_.store(0);
  home_fault_joint_.store(-1);home_fault_value_.store(0.);home_fault_limit_.store(0.);
  home_fault_elapsed_.store(0.);home_fault_position_.store(0.);home_fault_velocity_.store(0.);
}

bool A6YGravityCompController::fail_home(HomeFault fault, int joint, double value, double limit,
                                       double position, double velocity)
{
  // Publish the complete first-failure snapshot before phase 3. No allocation or
  // ROS logging in the control loop; the status service formats these atomics.
  home_fault_joint_.store(joint);home_fault_value_.store(value);home_fault_limit_.store(limit);
  home_fault_elapsed_.store(home_elapsed_);home_fault_position_.store(position);
  home_fault_velocity_.store(velocity);home_fault_revision_.store(home_rt_revision_.load());
  home_fault_.store(fault);home_integral_.setZero();home_phase_.store(3);
  if(teaching_position_output_) for(int i=0;i<6;++i)
    position_target_[i]=std::clamp(q_[pinocchio_q_indices_[i]],home_lower_[i],home_upper_[i]);
  return false;
}

std::string A6YGravityCompController::home_fault_message()
{
  const auto fault=home_fault_.load();const auto revision=home_fault_revision_.load();
  if(fault==HomeFault::None || revision!=home_revision_.load()) return "OK";
  const char* reason="unknown";
  switch(fault) {
    case HomeFault::None: break;
    case HomeFault::StartPosition: reason="start_position_mismatch";break;
    case HomeFault::StartVelocity: reason="start_velocity";break;
    case HomeFault::ControlPeriod: reason="invalid_control_period";break;
    case HomeFault::Heartbeat: reason="heartbeat_timeout";break;
    case HomeFault::TrackingError: reason="tracking_error";break;
    case HomeFault::JointSpeed: reason="joint_speed";break;
    case HomeFault::LowerLimit: reason="joint_lower_limit";break;
    case HomeFault::UpperLimit: reason="joint_upper_limit";break;
    case HomeFault::Solver: reason="impedance_solver_invalid";break;
    case HomeFault::SettlingTimeout: reason="zero_settling_timeout";break;
  }
  std::ostringstream message;message<<std::fixed<<std::setprecision(6);
  message<<"Return aborted: "<<reason<<" revision="<<revision;
  const auto joint=home_fault_joint_.load();
  if(joint>=0 && joint<6) message<<" joint="<<joint_names_[joint];
  message<<" value="<<home_fault_value_.load()<<" limit="<<home_fault_limit_.load()
         <<" elapsed="<<home_fault_elapsed_.load()<<" s";
  if(joint>=0) message<<" position="<<home_fault_position_.load()<<" velocity="<<home_fault_velocity_.load();
  if(fault==HomeFault::ControlPeriod) message<<" (valid period: 0 < dt <= limit)";
  if(fault==HomeFault::Solver) message<<" (value=solver iterations, limit=iteration budget)";
  if(fault==HomeFault::SettlingTimeout) message<<" (value=seconds after path end; position tolerance=0.005 rad, velocity tolerance=0.03 rad/s, stable time=0.3 s)";
  const auto result=message.str();
  if(home_logged_failure_revision_!=revision) {
    RCLCPP_ERROR(get_node()->get_logger(),"%s",result.c_str());
    home_logged_failure_revision_=revision;
  }
  return result;
}

void A6YGravityCompController::reset_home()
{
  reset_follow();
  home_controller_active_.store(false);home_pending_.store(false);home_phase_.store(0);home_buffer_.reset();
  home_rt_revision_=0;home_elapsed_=home_settled_=0;home_integral_.setZero();clear_home_fault();
}

void A6YGravityCompController::accept_home(const HomeService::Request& r, HomeService::Response& out)
{
  out.success=false;
  // A cancel/retry may be the first callback after the RT loop aborts. Preserve
  // that latched fault in the log before advancing the request revision.
  if(home_phase_.load()==3) home_fault_message();
  if(r.command==0) {
    if(r.revision==home_revision_.load()) home_heartbeat_.store(staged_output_steady_time_seconds());
    out.success=true;
  } else if(r.command==2 && r.revision==home_revision_.load()) {
    GravityHomePath path;path.revision=++home_next_revision_;path.cancel=true;
    home_revision_.store(path.revision);home_buffer_.writeFromNonRT(path);out.success=true;
  } else if(r.command==1) {
    if(!home_controller_active_.load() || !enable_output_) out.message="Gravity controller is not active";
    else if(home_pending_.load() || home_phase_.load()==1 || home_phase_.load()==2 || follow_phase_.load()!=0 || follow_requested_.command!=0) out.message="Return/follow already in progress";
    else {
      GravityHomePath path;path.times=r.times;path.positions=r.positions;path.velocities=r.velocities;
      if(!path.valid(home_lower_,home_upper_)) out.message="Invalid/beyond-limit zero-return trajectory";
      else {
        path.revision=++home_next_revision_;home_revision_.store(path.revision);
        home_heartbeat_.store(staged_output_steady_time_seconds());home_pending_.store(true);
        home_buffer_.writeFromNonRT(path);out.success=true;
      }
    }
  } else out.message="Invalid command or stale revision";
  out.revision=home_revision_.load();out.phase=home_phase_.load();
  out.applied_revision=home_rt_revision_.load();
  if(out.message.empty()) out.message=out.phase==3?home_fault_message():"OK";
}

bool A6YGravityCompController::home_effort(double dt, double now, const Eigen::VectorXd& gravity, JointVector& effort)
{
  if(!home_enabled_) return false;
  const auto* path=home_buffer_.readFromRT();
  JointVector q,v;
  for(int i=0;i<6;++i) {q[i]=q_[pinocchio_q_indices_[i]];v[i]=dq_[pinocchio_v_indices_[i]];}
  if(path->revision && path->revision!=home_rt_revision_) {
    home_rt_revision_=path->revision;home_elapsed_=home_settled_=0;home_integral_.setZero();
    clear_home_fault();
    home_phase_.store(path->cancel?0:1);
    if(path->cancel && teaching_position_output_) position_target_=q;
    home_pending_.store(false);
    if(!path->cancel) for(int i=0;i<6;++i) {
      if(std::abs(q[i]-path->positions[i])>.03)
        return fail_home(HomeFault::StartPosition,i,std::abs(q[i]-path->positions[i]),.03,q[i],v[i]);
      if(std::abs(v[i])>.08) return fail_home(HomeFault::StartVelocity,i,std::abs(v[i]),.08,q[i],v[i]);
    }
  }
  if(home_phase_.load()!=1 && home_phase_.load()!=2) return false;
  if(!std::isfinite(dt) || dt<=0 || dt>.1) return fail_home(HomeFault::ControlPeriod,-1,dt,.1);
  const double heartbeat_age=now-home_heartbeat_.load();
  if(heartbeat_age>.6) return fail_home(HomeFault::Heartbeat,-1,heartbeat_age,.6);
  std::array<double,6> ref{},speed{};path->sample(home_elapsed_,ref,speed);
  JointVector error,relative_velocity,lower,upper,base;
  for(int i=0;i<6;++i) {
    error[i]=ref[i]-q[i];relative_velocity[i]=v[i]-speed[i];
    if(std::abs(error[i])>.15) return fail_home(HomeFault::TrackingError,i,std::abs(error[i]),.15,q[i],v[i]);
    if(std::abs(v[i])>feedback_velocity_limits_[i])
      return fail_home(HomeFault::JointSpeed,i,std::abs(v[i]),feedback_velocity_limits_[i],q[i],v[i]);
    if(q[i]<home_lower_[i]-.01) return fail_home(HomeFault::LowerLimit,i,q[i],home_lower_[i]-.01,q[i],v[i]);
    if(q[i]>home_upper_[i]+.01) return fail_home(HomeFault::UpperLimit,i,q[i],home_upper_[i]+.01,q[i],v[i]);
    base[i]=std::clamp(effort_signs_[i]*torque_scales_[i]*gravity[i],-torque_limits_[i],torque_limits_[i]);
  }
  if(!teaching_position_output_) {
  // J4's recorded stick/slip needs gentler position recovery. Other axes keep
  // their existing gains. Friction assist remains disabled during return.
  const JointVector stiffness((JointVector()<<54.,50.,54.,22.5,38.5,5.2).finished());
  const JointVector damping((JointVector()<<6.,4.5,3.6,1.8,1.8,.52).finished());
  const JointVector budget((JointVector()<<2.4,2.8,3.6,.6,.6,.6).finished());
  const JointVector ki((JointVector()<<4.,4.,8.,.8,1.2,1.).finished());
  const JointVector imax((JointVector()<<1.5,2.2,3.,.4,.4,.35).finished());
  for(int i=0;i<6;++i) {
    lower[i]=std::max(-budget[i],-torque_limits_[i]-base[i]);upper[i]=std::min(budget[i],torque_limits_[i]-base[i]);
    if(std::abs(error[i])>.003 && error[i]*home_integral_[i]<0) home_integral_[i]=0;
    home_integral_[i]=std::clamp(home_integral_[i],lower[i],upper[i]);
  }
  pinocchio::crba(*pinocchio_model_,*pinocchio_data_,q_);
  pinocchio_data_->M.template triangularView<Eigen::StrictlyLower>()=pinocchio_data_->M.transpose().template triangularView<Eigen::StrictlyLower>();
  JointMatrix mass;
  for(int i=0;i<6;++i) for(int j=0;j<6;++j) mass(i,j)=pinocchio_data_->M(pinocchio_v_indices_[i],pinocchio_v_indices_[j]);
  const double ramp=std::min(1.,home_elapsed_/2.);
  const auto step=bounded_joint_impedance(mass,error,relative_velocity,ramp*stiffness,damping,dt,lower-home_integral_,upper-home_integral_);
  if(!step.valid) return fail_home(HomeFault::Solver,-1,step.iterations,96.);
  for(int i=0;i<6;++i) {
    // J4's direct soft brake takes priority over spring/integral chasing.
    // Do not apply this extra damping indiscriminately to lower-inertia axes.
    // The raw hard-speed guard above still covers every axis.
    const auto brake=i==3 ? gravity_home_brake(home_integral_[i]+step.torque[i],v[i],lower[i],upper[i]) :
      GravityHomeBrakeResult{true,false,home_integral_[i]+step.torque[i]};
    if(!brake.valid) return fail_home(HomeFault::Solver,i,step.iterations,96.,q[i],v[i]);
    if(brake.braking && home_integral_[i]*v[i]>0) home_integral_[i]=0.;
    effort[i]=std::clamp(base[i]+brake.torque,-torque_limits_[i],torque_limits_[i]);
    if(!brake.braking && ramp>=1 && std::abs(error[i])>.003 && std::abs(v[i])<=.15 && !(step.saturated[i] && error[i]*(home_integral_[i]+step.torque[i])>=0))
      home_integral_[i]=std::clamp(home_integral_[i]+dt*ki[i]*error[i],-imax[i],imax[i]);
  }
  }
  home_elapsed_+=dt;
  if(home_elapsed_>=path->times.back()) {
    home_settled_=(q.cwiseAbs().maxCoeff()<=.005 && v.cwiseAbs().maxCoeff()<=.03)?home_settled_+dt:0.;
    if(home_settled_>=.3) home_phase_.store(2);
    else if(home_elapsed_>path->times.back()+10.) {
      // Position and speed both participate in settling. Report the joint
      // furthest outside either tolerance, including a moving near-zero axis.
      Eigen::Index joint=0;
      q.cwiseAbs().cwiseQuotient(JointVector::Constant(.005))
        .cwiseMax(v.cwiseAbs().cwiseQuotient(JointVector::Constant(.03))).maxCoeff(&joint);
      return fail_home(HomeFault::SettlingTimeout,static_cast<int>(joint),
                       home_elapsed_-path->times.back(),10.,q[joint],v[joint]);
    }
  }
  return true;
}
}
