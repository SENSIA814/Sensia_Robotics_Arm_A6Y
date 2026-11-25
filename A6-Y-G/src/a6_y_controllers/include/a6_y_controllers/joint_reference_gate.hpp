#pragma once

#include <a6_y_teleop_msgs/msg/vr_control.hpp>
#include <a6_y_teleop_msgs/msg/vr_joint_reference.hpp>
#include <algorithm>
#include <array>
#include <cmath>
#include <cstdint>

namespace a6_y_controllers
{
// No ROS calls, memory allocation or I/O in step(). Time is supplied by the caller.
class JointReferenceGate
{
public:
  using Joints = std::array<double, 6>;
  using Reference = a6_y_teleop_msgs::msg::VrJointReference;
  using Control = a6_y_teleop_msgs::msg::VrControl;
  enum State : uint8_t { DISABLED, HOLD, ARMED_HOLD, TELEOP, STOPPING, PAUSED, FAULT };
  enum Reason : uint8_t {
    NONE, NOT_PREPARED, PREPARED, ARMED, FOLLOWING, RELEASED, INPUT_TIMEOUT,
    REFERENCE_TIMEOUT, INVALID_REFERENCE, SESSION_CHANGED, MISSED_EDGE,
    TRACKING_ERROR, ACTUAL_LIMIT, ACTUAL_SPEED, STOP_TIMEOUT, STOPPED,
    PAUSE_REQUEST, DISARM_REQUEST, BAD_FEEDBACK, BAD_PERIOD, REQUEST_REJECTED,
    WAITING_FIRST_REFERENCE, SATURATION, COMPUTE_FAILURE, SESSION_TRAVEL_LIMIT, AUTO_REARMED,
    TRACKING_LIMITED
  };
  struct Config {
    Joints lower{{-2.62,0.,0.,-1.55,-.9,-3.14}};
    Joints upper{{2.62,3.16,3.16,1.55,.9,3.14}};
    Joints zero_tolerance{};
    Joints speed{{.1,.1,.1,.1,.1,.1}};
    Joints acceleration{{.3,.3,.3,.3,.3,.3}};
    Joints actual_speed{{.25,.25,.25,.25,.25,.25}};
    double tracking_error{.05}, feedback_tolerance{.005};
    double input_timeout{.15}, reference_timeout{.05}, max_period{.04};
    double settle_speed{.03}, settle_error{.02}, settle_duration{.15};
    double stop_timeout{2.}, request_timeout{.5}, capture_timeout{.15};
    double session_reference_travel{0.}, session_actual_travel{0.};
    bool allow_prepare{false}, allow_test_input{false}, allow_arm{true};
    bool auto_rearm_after_release{false};
    // Native-position goals may be farther ahead than an executable setpoint.
    // The local trajectory limiter still bounds every output and tracking gap.
    bool trajectory_goals{false};
    bool tracking_governor{false};
    bool gripper_control{false};
    double gripper_lower{-.03}, gripper_upper{0.}, gripper_zero_tolerance{0.};
    double gripper_speed{.003}, gripper_acceleration{.01}, gripper_actual_speed{.01};
    double gripper_tracking_error{.004}, gripper_settle_speed{.001}, gripper_settle_error{.0015};
    bool valid() const {
      for (size_t i=0;i<6;++i)
        if (!std::isfinite(lower[i]) || !std::isfinite(upper[i]) || lower[i]>=upper[i] ||
            !std::isfinite(zero_tolerance[i]) || zero_tolerance[i]<0 ||
            !positive(speed[i]) || !positive(acceleration[i]) || !positive(actual_speed[i]) ||
            speed[i]>actual_speed[i]) return false;
      const bool session_valid=(session_reference_travel==0. && session_actual_travel==0.) ||
        (positive(session_reference_travel) && positive(session_actual_travel) && session_reference_travel<session_actual_travel);
      const bool gripper_valid=!gripper_control || (std::isfinite(gripper_lower) && std::isfinite(gripper_upper) &&
        gripper_lower<gripper_upper && std::isfinite(gripper_zero_tolerance) && gripper_zero_tolerance>=0 &&
        positive(gripper_speed) && positive(gripper_acceleration) && positive(gripper_actual_speed) &&
        gripper_speed<=gripper_actual_speed && positive(gripper_tracking_error) &&
        positive(gripper_settle_speed) && positive(gripper_settle_error));
      return (!tracking_governor || trajectory_goals) && gripper_valid && session_valid && positive(tracking_error) && std::isfinite(feedback_tolerance) && feedback_tolerance>=0 &&
        positive(input_timeout) && positive(reference_timeout) && positive(max_period) &&
        positive(settle_speed) && positive(settle_error) && positive(settle_duration) &&
        positive(stop_timeout) && positive(request_timeout) && positive(capture_timeout);
    }
    static bool positive(double x) { return std::isfinite(x) && x>0; }
  };

  explicit JointReferenceGate(const Config& config) : config_(config) {}
  void reset(uint64_t epoch, uint64_t clock) {
    epoch_=epoch; clock_=clock; state_=DISABLED; reason_=NOT_PREPARED;
    takeover_=producer_=session_=accepted_=last_request_=0;
    have_input_=first_reference_=request_accepted_=auto_rearm_pending_=false;
    have_session_origin_=false;session_origin_.fill(0.);
    target_.fill(0); velocity_.fill(0); anchor_.fill(0);
    gripper_target_=gripper_anchor_=gripper_velocity_=gripper_actual_=0.;
    last_input_=Reference(); anchor_time_=stop_start_=settled_since_=0;
  }
  void fault(Reason why) { state_=FAULT; reason_=why; velocity_.fill(0); auto_rearm_pending_=false; ++takeover_; }
  void request_stop(const Joints& measured, uint64_t now, Reason why, bool normal=false) {
    if (state_==DISABLED || state_==FAULT) return;
    const bool recoverable=why==INPUT_TIMEOUT || why==REFERENCE_TIMEOUT ||
      why==INVALID_REFERENCE || why==MISSED_EDGE;
    if (state_!=STOPPING) {
      auto_rearm_pending_=config_.auto_rearm_after_release && recoverable &&
        (state_==ARMED_HOLD || state_==TELEOP);
      target_=measured; velocity_.fill(0); stop_start_=now; settled_since_=0;
      gripper_target_=gripper_actual_;gripper_velocity_=0.;
      state_=STOPPING; normal_stop_=normal; reason_=why; ++takeover_;
    } else if (!normal) {
      auto_rearm_pending_=config_.auto_rearm_after_release && recoverable &&
        (auto_rearm_pending_ || normal_stop_);
      normal_stop_=false; reason_=why;
    }
  }

  void step(uint64_t now, double dt, const Joints& q, const Joints& dq,
            bool feedback_valid, const Reference* incoming, const Control* command,
            uint8_t urgent_action=0, bool ready_to_arm=true, bool auxiliary_position_valid=true,
            double gripper_q=0., double gripper_dq=0.) {
    if (state_==FAULT) return;
    if (!std::isfinite(dt) || dt<=0 || dt>config_.max_period) { fault(BAD_PERIOD); return; }
    if (!feedback_valid || !finite(q) || !finite(dq)) { fault(BAD_FEEDBACK); return; }
    gripper_actual_=gripper_q;
    if(config_.gripper_control && (!std::isfinite(gripper_q) || !std::isfinite(gripper_dq))) {fault(BAD_FEEDBACK);return;}
    const bool gripper_in_bounds=!config_.gripper_control || position_allowed(gripper_q,
      config_.gripper_lower,config_.gripper_upper,config_.gripper_zero_tolerance);
    const bool gripper_stationary=!config_.gripper_control || std::abs(gripper_dq)<=config_.gripper_settle_speed;
    const bool gripper_settled=gripper_stationary && (!config_.gripper_control ||
      std::abs(gripper_q-gripper_target_)<=config_.gripper_settle_error);
    // When disabled, an out-of-range posture blocks prepare but does not move the arm.
    if (enabled() && (!within_actual(q) || !auxiliary_position_valid || !gripper_in_bounds)) { fault(ACTUAL_LIMIT); return; }
    if(enabled() && config_.gripper_control && std::abs(gripper_dq)>config_.gripper_actual_speed) {fault(ACTUAL_SPEED);return;}
    if (enabled() && !within_session(q,config_.session_actual_travel)) { fault(SESSION_TRAVEL_LIMIT); return; }
    if (enabled()) for (size_t i=0;i<6;++i)
      if (std::abs(dq[i])>config_.actual_speed[i]) { fault(ACTUAL_SPEED); return; }

    const bool was_prepared=enabled();
    if (urgent_action==Control::DISARM) {
      disable();
      if (command && command->action==Control::DISARM && command->controller_epoch==epoch_ &&
          age_ok(now,command->generated_monotonic_ns,config_.request_timeout)) {
        last_request_=command->request_id; request_accepted_=true;
      }
      return;
    }
    if (urgent_action==Control::PAUSE && enabled()) request_stop(q,now,PAUSE_REQUEST);

    bool new_input=false, input_order_ok=true;
    if (incoming && (!have_input_ || incoming->producer_id!=last_input_.producer_id ||
                      incoming->session_id!=last_input_.session_id || incoming->sequence>last_input_.sequence)) {
      const auto& r=*incoming;
      const bool bound=state_==ARMED_HOLD || state_==TELEOP || state_==STOPPING;
      if (bound && (r.producer_id!=producer_ || r.session_id!=session_)) {
        request_stop(q,now,SESSION_CHANGED);
      } else if (!valid_input(r,now)) {
        if (enabled() && state_!=HOLD && state_!=PAUSED) request_stop(q,now,INVALID_REFERENCE);
      } else {
        if (have_input_ && r.producer_id==last_input_.producer_id && r.session_id==last_input_.session_id) {
          const auto& old=last_input_;
          const bool order=r.generated_monotonic_ns<old.generated_monotonic_ns ||
            r.feedback_monotonic_ns<old.feedback_monotonic_ns || r.pose_sequence<old.pose_sequence || r.pose_monotonic_ns<old.pose_monotonic_ns ||
            r.buttons_monotonic_ns<old.buttons_monotonic_ns || r.clutch_epoch<old.clutch_epoch ||
            (r.pose_sequence==old.pose_sequence && r.pose_monotonic_ns!=old.pose_monotonic_ns);
          input_order_ok=!order;
          const uint64_t edges=r.clutch_epoch>=old.clutch_epoch ? r.clutch_epoch-old.clutch_epoch : 0;
          if (order || edges>1 || ((edges==1)!=(r.squeeze!=old.squeeze))) {
            if (bound) request_stop(q,now,order ? INVALID_REFERENCE : MISSED_EDGE);
          }
        }
        last_input_=r; have_input_=true; new_input=true;
        if (state_==TELEOP && !r.squeeze) request_stop(q,now,RELEASED,true);
      }
    }

    if (command && command->request_id>last_request_) {
      last_request_=command->request_id; request_accepted_=false;
      const auto& c=*command;
      if (c.controller_epoch==epoch_ && age_ok(now,c.generated_monotonic_ns,config_.request_timeout)) {
        if (c.action==Control::DISARM) { disable(); request_accepted_=true; return; }
        if (c.action==Control::PAUSE && enabled()) {
          request_stop(q,now,PAUSE_REQUEST); request_accepted_=true;
        } else if (c.action==Control::PREPARE && state_==DISABLED && config_.allow_prepare && within_actual(q) &&
                   auxiliary_position_valid && gripper_in_bounds && gripper_stationary &&
                   within_session(q,config_.session_actual_travel) && max_abs(dq)<=config_.settle_speed) {
          if(!have_session_origin_) {session_origin_=q;have_session_origin_=true;}
          target_=anchor_=q; velocity_.fill(0); state_=HOLD; reason_=PREPARED;
          gripper_target_=gripper_anchor_=gripper_q;gripper_velocity_=0.;
          request_accepted_=true; ++takeover_;
        } else if (c.action==Control::ARM && config_.allow_arm && ready_to_arm &&
                   within_session(q,config_.session_reference_travel) && (state_==HOLD || state_==PAUSED) && have_input_ &&
                   valid_input(last_input_,now) && !last_input_.squeeze &&
                   c.producer_id==last_input_.producer_id && c.session_id==last_input_.session_id &&
                   gripper_settled && max_abs(dq)<=config_.settle_speed && max_error(q,target_)<=config_.settle_error) {
          producer_=c.producer_id; session_=c.session_id; state_=ARMED_HOLD; reason_=ARMED;
          auto_rearm_pending_=false;
          armed_epoch_=last_input_.clutch_epoch; request_accepted_=true; ++takeover_;
        } else if (c.action==Control::CAPTURE && config_.allow_arm && within_session(q,config_.session_reference_travel) &&
                   state_==ARMED_HOLD && have_input_ &&
                   valid_input(last_input_,now) && last_input_.squeeze &&
                   c.producer_id==producer_ && c.session_id==session_ &&
                   c.clutch_epoch==last_input_.clutch_epoch && c.clutch_epoch==armed_epoch_+1 &&
                   gripper_settled && max_abs(dq)<=config_.settle_speed && max_error(q,target_)<=config_.settle_error) {
          target_=anchor_=q; velocity_.fill(0); anchor_time_=now; ++takeover_;
          gripper_target_=gripper_anchor_=gripper_q;gripper_velocity_=0.;
          first_reference_=true; state_=TELEOP; reason_=WAITING_FIRST_REFERENCE; request_accepted_=true;
        }
      }
    }

    // Recover only a previously armed stream. A fresh released grip after the
    // stop restores permission; a subsequent press still performs CAPTURE at
    // the actual arm/gripper pose. Never undo local PAUSE, DISARM or FAULT.
    if (state_==PAUSED && auto_rearm_pending_ && config_.allow_arm && ready_to_arm &&
        new_input && input_order_ok && valid_input(last_input_,now) && !last_input_.squeeze &&
        last_input_.producer_id==producer_ && last_input_.session_id==session_ &&
        last_input_.pose_monotonic_ns>stop_start_ && last_input_.buttons_monotonic_ns>stop_start_ &&
        within_session(q,config_.session_reference_travel) && gripper_settled &&
        max_abs(dq)<=config_.settle_speed && max_error(q,target_)<=config_.settle_error) {
      state_=ARMED_HOLD;reason_=AUTO_REARMED;armed_epoch_=last_input_.clutch_epoch;
      auto_rearm_pending_=false;++takeover_;
    }

    if (state_==ARMED_HOLD && have_input_ && !last_input_.squeeze)
      armed_epoch_=last_input_.clutch_epoch;
    if (state_==ARMED_HOLD || state_==TELEOP) {
      if (!have_input_ || !age_ok(now,last_input_.pose_monotonic_ns,config_.input_timeout) ||
          !age_ok(now,last_input_.buttons_monotonic_ns,config_.input_timeout))
        request_stop(q,now,INPUT_TIMEOUT);
      else if (!age_ok(now,last_input_.generated_monotonic_ns,config_.reference_timeout))
        request_stop(q,now,REFERENCE_TIMEOUT);
    }
    if (state_==TELEOP) {
      if (max_error(q,target_)>config_.tracking_error || (config_.gripper_control &&
          std::abs(gripper_q-gripper_target_)>config_.gripper_tracking_error)) request_stop(q,now,TRACKING_ERROR);
      else if (first_reference_ && !age_ok(now,anchor_time_,config_.capture_timeout))
        request_stop(q,now,REFERENCE_TIMEOUT);
      else if (new_input && last_input_.takeover_id==takeover_) {
        if (first_reference_) {
          if (max_error(last_input_.positions,anchor_)>1e-8 || (config_.gripper_control &&
              std::abs(last_input_.gripper_position-gripper_anchor_)>1e-8)) request_stop(q,now,INVALID_REFERENCE);
          else { first_reference_=false; accepted_=last_input_.sequence; reason_=FOLLOWING; }
        } else {
          auto& desired=last_input_.positions;
          const double feedback_delay=double(now-last_input_.feedback_monotonic_ns)*1e-9;
          bool valid=true;
          for (size_t i=0;i<6;++i) {
            project_delayed_zero_return(desired[i],target_[i],config_.lower[i],config_.upper[i],
              config_.zero_tolerance[i],config_.speed[i]*feedback_delay);
            valid &= within_reference(i,desired[i]) &&
              (config_.trajectory_goals || std::abs(desired[i]-q[i])<=config_.tracking_error);
          }
          if(config_.gripper_control) {
            project_delayed_zero_return(last_input_.gripper_position,gripper_target_,
              config_.gripper_lower,config_.gripper_upper,config_.gripper_zero_tolerance,
              config_.gripper_speed*feedback_delay);
            valid &= within_gripper_reference(last_input_.gripper_position) &&
              (config_.trajectory_goals ||
               std::abs(last_input_.gripper_position-gripper_q)<=config_.gripper_tracking_error);
          }
          if (!valid) request_stop(q,now,TRACKING_ERROR);
        }
      }
      // Only the current takeover's reference may advance the local limiter.
      if (state_==TELEOP && !first_reference_ && last_input_.takeover_id==takeover_) {
        Joints next=target_, next_velocity=velocity_;
        bool valid_candidate=true;
        bool tracking_limited=false;
        for (size_t i=0;i<6;++i) {
          // A previously accepted goal may lag this limiter as it brakes while
          // returning from tolerated zero error. Hold the moving boundary;
          // never reverse back out of the nominal range to chase an old goal.
          const double desired=std::clamp(last_input_.positions[i],
            std::min(config_.lower[i],target_[i]),std::max(config_.upper[i],target_[i]));
          const double error=desired-target_[i];
          const double braking=braking_speed(std::abs(error),config_.acceleration[i],dt);
          double demand=std::copysign(std::min(config_.speed[i],braking),error);
          if(config_.tracking_governor) {
            const double requested=demand;
            demand=tracking_demand(demand,target_[i]-q[i],dq[i],config_.tracking_error,
                                   config_.acceleration[i],dt);
            tracking_limited |= std::abs(demand)<std::abs(requested)-1e-12;
          }
          const double a=config_.acceleration[i]*dt;
          next_velocity[i]=std::clamp(demand,velocity_[i]-a,velocity_[i]+a);
          next[i]+=next_velocity[i]*dt;
          // At a target crossing only settle if the acceleration constraint permits it.
          if (std::abs(error)<=std::abs(next_velocity[i]*dt) &&
              error*next_velocity[i]>=0 && std::abs(velocity_[i])<=a) {
            next[i]=desired; next_velocity[i]=(next[i]-target_[i])/dt;
          }
          if (!within_reference(i,next[i]) ||
              std::abs(next[i]-q[i])>config_.tracking_error) valid_candidate=false;
        }
        double next_gripper=gripper_target_, next_gripper_velocity=gripper_velocity_;
        if(config_.gripper_control) {
          const double desired=std::clamp(last_input_.gripper_position,
            std::min(config_.gripper_lower,gripper_target_),std::max(config_.gripper_upper,gripper_target_));
          const double error=desired-gripper_target_;
          double demand=std::copysign(std::min(config_.gripper_speed,
            braking_speed(std::abs(error),config_.gripper_acceleration,dt)),error);
          if(config_.tracking_governor) {
            const double requested=demand;
            demand=tracking_demand(demand,gripper_target_-gripper_q,gripper_dq,
                                   config_.gripper_tracking_error,config_.gripper_acceleration,dt);
            tracking_limited |= std::abs(demand)<std::abs(requested)-1e-12;
          }
          const double a=config_.gripper_acceleration*dt;
          next_gripper_velocity=std::clamp(demand,gripper_velocity_-a,gripper_velocity_+a);
          next_gripper+=next_gripper_velocity*dt;
          if(std::abs(error)<=std::abs(next_gripper_velocity*dt) && error*next_gripper_velocity>=0 && std::abs(gripper_velocity_)<=a) {
            next_gripper=desired;next_gripper_velocity=(next_gripper-gripper_target_)/dt;
          }
          valid_candidate &= within_gripper_reference(next_gripper) &&
            std::abs(next_gripper-gripper_q)<=config_.gripper_tracking_error;
        }
        if (!valid_candidate) { request_stop(q,now,TRACKING_ERROR); }
        else { target_=next; velocity_=next_velocity; gripper_target_=next_gripper;
          gripper_velocity_=next_gripper_velocity;accepted_=last_input_.sequence;
          reason_=tracking_limited?TRACKING_LIMITED:FOLLOWING; }
      }
    }
    if (state_==STOPPING) {
      const bool settled=gripper_stationary && (!config_.gripper_control ||
        std::abs(gripper_q-gripper_target_)<=config_.gripper_settle_error) &&
        max_abs(dq)<=config_.settle_speed && max_error(q,target_)<=config_.settle_error;
      if (!settled) settled_since_=0;
      else if (!settled_since_) settled_since_=now;
      if (settled_since_ && now-settled_since_>=uint64_t(config_.settle_duration*1e9)) {
        state_=normal_stop_ ? ARMED_HOLD : PAUSED; reason_=STOPPED;
        if (normal_stop_ && have_input_) armed_epoch_=last_input_.clutch_epoch;
      } else if (now-stop_start_>uint64_t(config_.stop_timeout*1e9)) fault(STOP_TIMEOUT);
    }
    // Tracks entry only; the controller owns the effort activation ramp.
    just_prepared_=!was_prepared && enabled();
  }

  bool enabled() const { return state_!=DISABLED && state_!=FAULT; }
  State state() const { return state_; }
  Reason reason() const { return reason_; }
  const Joints& target() const { return target_; }
  const Joints& velocity() const { return velocity_; }
  const Joints& anchor() const { return anchor_; }
  bool gripper_control() const { return config_.gripper_control; }
  double gripper_target() const { return gripper_target_; }
  double gripper_anchor() const { return gripper_anchor_; }
  double gripper_speed() const { return config_.gripper_speed; }
  uint64_t epoch() const { return epoch_; }
  uint64_t takeover() const { return takeover_; }
  uint64_t producer() const { return producer_; }
  uint64_t session() const { return session_; }
  uint64_t accepted() const { return accepted_; }
  uint64_t anchor_time() const { return anchor_time_; }
  uint64_t request_id() const { return last_request_; }
  bool request_accepted() const { return request_accepted_; }
  bool just_prepared() const { return just_prepared_; }
  bool follow_permitted() const { return config_.allow_prepare && config_.allow_arm; }
  bool auto_rearm_after_release() const { return config_.auto_rearm_after_release; }
  double session_reference_travel() const { return config_.session_reference_travel; }
  double session_actual_travel() const { return config_.session_actual_travel; }
  const Joints& session_origin() const { return session_origin_; }
  static bool age_ok(uint64_t now,uint64_t stamp,double limit) {
    return stamp>0 && stamp<=now && now-stamp<=uint64_t(limit*1e9);
  }
  static bool position_allowed(double q,double lower,double upper,double zero_tolerance,double tolerance=0.) {
    return std::isfinite(q) &&
      q>=lower-std::max(tolerance,lower==0. ? zero_tolerance : 0.) &&
      q<=upper+std::max(tolerance,upper==0. ? zero_tolerance : 0.);
  }
private:
  static double tracking_demand(double demand,double gap,double measured_velocity,
                                double tracking_error,double acceleration,double dt) {
    // Reserve braking distance for closing velocity relative to the measured
    // joint, not absolute command speed. A well-tracking joint at 1 rad/s has
    // almost no closing velocity, so this does not recreate a static speed cap.
    // The remaining 20% of the hard tracking envelope absorbs feedback
    // quantization / response changes; the independent hard check stays active.
    const double direction=demand<0. ? -1. : 1.;
    // Keep the correction signed: beyond the soft envelope, matching the
    // measured speed only preserves the gap. Quantized / biased velocity and
    // finite deceleration can then accumulate lead until the hard stop trips.
    // Slow below measured speed to recover margin, without reversing a goal
    // or jumping position/velocity. The caller still bounds acceleration.
    const double room=.8*tracking_error-direction*gap;
    const double correction=std::copysign(braking_speed(std::abs(room),acceleration,dt),room);
    const double allowed=std::max(0.,direction*measured_velocity+
      correction);
    return direction*std::min(std::abs(demand),allowed);
  }
  static double braking_speed(double distance,double acceleration,double dt) {
    // Reserve this update's travel as well as the subsequent braking distance:
    // v*dt + v*v/(2*a) <= distance. sqrt(2*a*distance) alone can cross a hard
    // bound by one discrete step, then incorrectly report TRACKING_ERROR.
    // Rationalized form avoids cancellation as distance approaches zero.
    const double a_step=acceleration*dt,numerator=2*acceleration*distance;
    return numerator/(std::sqrt(a_step*a_step+numerator)+a_step);
  }
  static void project_delayed_zero_return(double& desired,double target,double lower,double upper,
                                         double zero_tolerance,double maximum_advance) {
    // The bridge uses a timestamped older controller state. At 125 Hz, its
    // valid inward/hold goal can arrive behind the current zero-return bound.
    // Accept only lag explainable by the configured speed and feedback age,
    // and project inward. This grants no extra outward command travel.
    const double lo=std::min(lower,target),hi=std::max(upper,target);
    if(lower==0. && desired<lo && desired>=lower-zero_tolerance &&
        lo-desired<=maximum_advance+1e-12) desired=lo;
    else if(upper==0. && desired>hi && desired<=upper+zero_tolerance &&
        desired-hi<=maximum_advance+1e-12) desired=hi;
  }
  bool within_gripper_reference(double q) const {
    return std::isfinite(q) && q>=std::min(config_.gripper_lower,gripper_target_) &&
      q<=std::max(config_.gripper_upper,gripper_target_);
  }
  static bool finite(const Joints& a) { for(double x:a) if(!std::isfinite(x)) return false; return true; }
  static double max_abs(const Joints& a) { double m=0;for(double x:a)m=std::max(m,std::abs(x));return m; }
  static double max_error(const Joints& a,const Joints& b) {
    double m=0;for(size_t i=0;i<6;++i)m=std::max(m,std::abs(a[i]-b[i]));return m;
  }
  bool within_actual(const Joints& q) const {
    for(size_t i=0;i<6;++i)
      if(!position_allowed(q[i],config_.lower[i],config_.upper[i],config_.zero_tolerance[i],config_.feedback_tolerance)) return false;
    return true;
  }
  bool within_reference(size_t i,double q) const {
    // Preserve a tolerated measured anchor. Each adopted step can only hold or
    // return toward nominal limits; the tolerance is not extra command travel.
    return q>=std::min(config_.lower[i],target_[i]) && q<=std::max(config_.upper[i],target_[i]) &&
      (config_.session_reference_travel==0. || std::abs(q-session_origin_[i])<=config_.session_reference_travel);
  }
  bool within_session(const Joints& q,double limit) const {
    return !have_session_origin_ || limit==0. || max_error(q,session_origin_)<=limit;
  }
  bool valid_input(const Reference& r,uint64_t now) const {
    return r.clock_id==clock_ && r.controller_epoch==epoch_ && r.valid &&
      (r.live || config_.allow_test_input) && r.producer_id && r.session_id && r.sequence &&
      finite(r.positions) && (!config_.gripper_control || (r.gripper_control && std::isfinite(r.gripper_position))) &&
      age_ok(now,r.generated_monotonic_ns,config_.reference_timeout) &&
      age_ok(now,r.feedback_monotonic_ns,config_.reference_timeout) &&
      age_ok(now,r.pose_monotonic_ns,config_.input_timeout) &&
      age_ok(now,r.buttons_monotonic_ns,config_.input_timeout);
  }
  void disable() {
    state_=DISABLED;reason_=DISARM_REQUEST;velocity_.fill(0);++takeover_;
    have_input_=false;producer_=session_=0;just_prepared_=false;
    auto_rearm_pending_=false;
  }
  Config config_;
  State state_{DISABLED};Reason reason_{NOT_PREPARED};
  Joints target_{},velocity_{},anchor_{},session_origin_{};
  double gripper_target_{0.},gripper_anchor_{0.},gripper_velocity_{0.},gripper_actual_{0.};
  Reference last_input_{};
  uint64_t epoch_{0},clock_{0},takeover_{0},producer_{0},session_{0},accepted_{0};
  uint64_t armed_epoch_{0},last_request_{0},anchor_time_{0},stop_start_{0},settled_since_{0};
  bool have_input_{false},first_reference_{false},normal_stop_{false},request_accepted_{false};
  bool just_prepared_{false};
  bool have_session_origin_{false};
  bool auto_rearm_pending_{false};
};
}  // namespace a6_y_controllers
