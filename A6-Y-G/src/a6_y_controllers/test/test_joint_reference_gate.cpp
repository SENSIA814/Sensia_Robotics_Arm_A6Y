#include <a6_y_controllers/joint_reference_gate.hpp>
#include <gtest/gtest.h>
#include <limits>

using Gate=a6_y_controllers::JointReferenceGate;
struct Rig {
  Gate::Config cfg;
  Gate gate;
  Gate::Joints q{{.2,.8,1.1,-.7,.4,1.}},dq{};
  Gate::Reference ref;
  Gate::Control control;
  uint64_t now{1000000000000ULL},seq{0},request{0},edge{0};
  bool held{false};
  double gripper_q{-.015},gripper_dq{0.};
  static Gate::Config config(bool allow=true) {Gate::Config c;c.allow_prepare=allow;return c;}
  explicit Rig(bool allow=true):cfg(config(allow)),gate(cfg) {gate.reset(11,22);}
  explicit Rig(const Gate::Config& config):cfg(config),gate(cfg) {gate.reset(11,22);}
  void input(bool grip=false) {
    if(grip!=held) ++edge;
    held=grip;
    ref.clock_id=22;ref.controller_epoch=11;ref.producer_id=33;ref.session_id=44;
    ref.takeover_id=gate.takeover();ref.sequence=++seq;ref.pose_sequence=seq;ref.clutch_epoch=edge;
    ref.generated_monotonic_ns=ref.pose_monotonic_ns=ref.buttons_monotonic_ns=ref.feedback_monotonic_ns=now;
    ref.valid=ref.live=true;ref.squeeze=grip;ref.positions=gate.state()==Gate::DISABLED?q:gate.target();
    ref.gripper_control=cfg.gripper_control;
    ref.gripper_position=gate.enabled()?gate.gripper_target():gripper_q;
  }
  void tick(bool receive=true,const Gate::Control* c=nullptr) {
    gate.step(now,.008,q,dq,true,receive?&ref:nullptr,c,0,true,true,gripper_q,gripper_dq);now+=8000000;
  }
  void command(uint8_t action) {
    control.controller_epoch=11;control.request_id=++request;control.generated_monotonic_ns=now;
    control.producer_id=33;control.session_id=44;control.clutch_epoch=edge;control.action=action;
    tick(true,&control);
  }
  void prepare() {input();command(Gate::Control::PREPARE);ASSERT_EQ(gate.state(),Gate::HOLD);}
  void arm() {input();command(Gate::Control::ARM);ASSERT_EQ(gate.state(),Gate::ARMED_HOLD);}
  void capture() {input(true);command(Gate::Control::CAPTURE);ASSERT_EQ(gate.state(),Gate::TELEOP);
    input(true);ref.positions=gate.anchor();tick();ASSERT_EQ(gate.reason(),Gate::FOLLOWING);}
  void follow() {prepare();arm();capture();}
};

TEST(JointReferenceGate, DisabledByDefaultAndExplicitPrepareCannotBypassConfiguration) {
  Rig blocked(false);blocked.input();blocked.command(Gate::Control::PREPARE);
  EXPECT_FALSE(blocked.gate.enabled());EXPECT_FALSE(blocked.gate.request_accepted());
  Rig r;r.input(true);r.tick();EXPECT_EQ(r.gate.state(),Gate::DISABLED);
  r.prepare();EXPECT_EQ(r.gate.target(),r.q);
}
TEST(JointReferenceGate, HeldGripCannotArmAndFirstReferenceMustMatchActualAnchor) {
  Rig r;r.prepare();r.input(true);r.command(Gate::Control::ARM);
  EXPECT_EQ(r.gate.state(),Gate::HOLD);
  r.input();r.arm();r.input(true);r.command(Gate::Control::CAPTURE);
  r.input(true);r.ref.positions[0]+=.01;r.tick();
  EXPECT_EQ(r.gate.state(),Gate::STOPPING);EXPECT_EQ(r.gate.target(),r.q);
}
TEST(JointReferenceGate, VelocityAndAccelerationStayBoundedAcrossChangingGoals) {
  Rig r;r.follow();auto previous=r.gate.target();Gate::Joints v{};
  for(int k=0;k<250;++k) {
    r.input(true);r.ref.positions[0]=r.q[0]+(k<150?.02:-.02);r.tick();
    ASSERT_EQ(r.gate.state(),Gate::TELEOP);
    for(size_t i=0;i<6;++i) {
      const double next_v=(r.gate.target()[i]-previous[i])/.008;
      EXPECT_LE(std::abs(next_v),.1+1e-10);
      EXPECT_LE(std::abs(next_v-v[i]),.3*.008+1e-10);
      v[i]=next_v;
    }
    previous=r.gate.target();
  }
}

static Gate::Config trajectory_goal_config(bool govern=true) {
  auto cfg=Rig::config();cfg.trajectory_goals=true;cfg.tracking_governor=govern;
  cfg.speed.fill(1.);cfg.acceleration.fill(3.);cfg.actual_speed.fill(1.25);
  return cfg;
}

#include "tracking_governor_fixture.hpp"
#include "tracking_drift_fixture.hpp"

TEST(JointReferenceGate, RecordedMultiSecondTrackingDriftDoesNotPause) {
  auto cfg=trajectory_goal_config();cfg.gripper_control=true;cfg.feedback_tolerance=0.;
  cfg.zero_tolerance.fill(.03490658503988659);cfg.gripper_zero_tolerance=.0008342673824532896;
  Rig r(cfg);r.q=tracking_drift_anchor;r.gripper_q=tracking_drift_gripper_anchor;r.follow();
  double peak=0.,last_gripper=r.gripper_q,last_gripper_velocity=0.;
  Gate::Joints last_velocity{};int samples=0;
  for(const auto& sample:tracking_drift_samples) {
    r.q=sample.q;r.dq=sample.dq;r.gripper_q=sample.gripper_q;r.gripper_dq=sample.gripper_dq;
    r.input(true);r.ref.positions=sample.goal;r.ref.gripper_position=sample.gripper_goal;
    r.gate.step(r.now,sample.dt,r.q,r.dq,true,&r.ref,nullptr,0,true,true,r.gripper_q,r.gripper_dq);
    r.now+=uint64_t(sample.dt*1e9);++samples;
    ASSERT_EQ(r.gate.state(),Gate::TELEOP)<<"sample="<<samples<<" reason="<<int(r.gate.reason());
    for(size_t i=0;i<6;++i) {
      const double gap=std::abs(r.gate.target()[i]-r.q[i]);peak=std::max(peak,gap);
      EXPECT_LE(gap,cfg.tracking_error);
      EXPECT_LE(std::abs(r.gate.velocity()[i]),cfg.speed[i]+1e-12);
      EXPECT_LE(std::abs(r.gate.velocity()[i]-last_velocity[i]),cfg.acceleration[i]*sample.dt+1e-12);
    }
    const double gv=(r.gate.gripper_target()-last_gripper)/sample.dt;
    EXPECT_LE(std::abs(gv),cfg.gripper_speed+1e-12);
    EXPECT_LE(std::abs(gv-last_gripper_velocity),cfg.gripper_acceleration*sample.dt+1e-12);
    EXPECT_LE(std::abs(r.gate.gripper_target()-r.gripper_q),cfg.gripper_tracking_error);
    last_velocity=r.gate.velocity();last_gripper=r.gate.gripper_target();last_gripper_velocity=gv;
  }
  EXPECT_EQ(samples,587);
  RecordProperty("replayed_samples",samples);RecordProperty("peak_gap_rad",std::to_string(peak));
}

TEST(JointReferenceGate, RecordedSlowFeedbackKeepsFollowingWithoutCrossingTrackingLimit) {
  auto cfg=trajectory_goal_config();cfg.zero_tolerance.fill(.03490658503988659);
  Rig r(cfg);r.q=tracking_anchor;r.follow();double peak_lead=0.;int limited=0;
  for(const auto& sample:tracking_samples) {
    r.q=sample.q;r.dq=sample.dq;r.input(true);r.ref.positions=sample.goal;
    r.gate.step(r.now,sample.dt,r.q,r.dq,true,&r.ref,nullptr);
    r.now+=uint64_t(sample.dt*1e9);
    ASSERT_EQ(r.gate.state(),Gate::TELEOP)<<"reason="<<int(r.gate.reason());
    if(r.gate.reason()==Gate::TRACKING_LIMITED)++limited;
    for(size_t i=0;i<6;++i) {
      EXPECT_LE(std::abs(r.gate.target()[i]-r.q[i]),cfg.tracking_error);
      peak_lead=std::max(peak_lead,std::abs(r.gate.target()[i]-r.q[i]));
    }
  }
  EXPECT_GT(limited,0);
  RecordProperty("peak_replayed_output_lead_rad",std::to_string(peak_lead));
  RecordProperty("limited_samples",limited);
}

TEST(JointReferenceGate, SlowQuantizedActuatorsRecoverSpeedWithoutRearming) {
  for(size_t axis=0;axis<6;++axis) for(double direction:{-1.,1.}) {
    const auto cfg=trajectory_goal_config();Rig r(cfg);
    r.q[axis]=(cfg.lower[axis]+cfg.upper[axis])*.5;r.follow();
    double plant=r.q[axis],previous_velocity=0.,late_peak=0.;
    for(int k=0;k<250;++k) {
      // 160 ms stiction, then slow motion, then improved response. Position
      // quantization matches the recorded motor's approximately .000381 rad.
      const double tau=k<125?.3:.04;
      const double motion=k<20?0.:(r.gate.target()[axis]-plant)*.008/tau;
      plant+=motion;r.q[axis]=std::round(plant/.000381475547417409)*.000381475547417409;
      r.dq[axis]=motion/.008;r.input(true);
      r.ref.positions[axis]=std::clamp(r.gate.target()[axis]+direction*.25,cfg.lower[axis],cfg.upper[axis]);
      r.tick();ASSERT_EQ(r.gate.state(),Gate::TELEOP)<<axis<<" tick="<<k<<" reason="<<int(r.gate.reason());
      const double v=r.gate.velocity()[axis];
      EXPECT_LE(std::abs(v),cfg.speed[axis]+1e-12);
      EXPECT_LE(std::abs(v-previous_velocity),cfg.acceleration[axis]*.008+1e-12);
      EXPECT_LE(std::abs(r.gate.target()[axis]-r.q[axis]),cfg.tracking_error);
      previous_velocity=v;if(k>125)late_peak=std::max(late_peak,std::abs(v));
    }
    EXPECT_GT(late_peak,.9)<<axis;
  }
}

TEST(JointReferenceGate, SmallReportedVelocityCannotAccumulateUnboundedPositionLead) {
  // The motor reports a nonzero velocity bin even when its quantized position
  // does not change. Matching that velocity forever must not consume the
  // remaining tracking margin and pause an otherwise valid VR stream.
  for(double direction:{-1.,1.}) {
    const auto cfg=trajectory_goal_config();Rig r(cfg);r.follow();
    double previous_v=0.;
    for(int k=0;k<1500;++k) {
      r.dq[0]=direction*.0024420024420024333;r.input(true);
      r.ref.positions[0]=r.q[0]+direction*.25;r.tick();
      ASSERT_EQ(r.gate.state(),Gate::TELEOP)<<"tick="<<k<<" reason="<<int(r.gate.reason());
      EXPECT_LE(std::abs(r.gate.target()[0]-r.q[0]),.041);
      EXPECT_LE(std::abs(r.gate.velocity()[0]-previous_v),cfg.acceleration[0]*.008+1e-12);
      previous_v=r.gate.velocity()[0];
    }
    EXPECT_NEAR(previous_v,0.,1e-10);
  }
}

TEST(JointReferenceGate, MovingFeedbackWithVelocityBiasRecoversTrackingMargin) {
  for(size_t axis=0;axis<6;++axis) for(double direction:{-1.,1.}) {
    const auto cfg=trajectory_goal_config();Rig r(cfg);
    r.q[axis]=(cfg.lower[axis]+cfg.upper[axis])*.5;r.follow();
    const double origin=r.q[axis];double previous_v=0.;
    for(int k=0;k<1000;++k) {
      // Position and velocity channels do not integrate exactly. Also include
      // a short stationary interval with a stale positive velocity estimate.
      const int moving_ticks=k<400?k:(k<430?400:k-30);
      r.q[axis]=std::round((origin+direction*.08*moving_ticks*.008)/.000381475547417409)*.000381475547417409;
      r.dq[axis]=direction*(.095+(k%2?.015:-.015));r.input(true);
      r.ref.positions[axis]=std::clamp(r.gate.target()[axis]+direction*.25,cfg.lower[axis],cfg.upper[axis]);
      r.tick();ASSERT_EQ(r.gate.state(),Gate::TELEOP)<<axis<<" "<<direction<<" tick="<<k;
      EXPECT_LE(std::abs(r.gate.target()[axis]-r.q[axis]),cfg.tracking_error);
      EXPECT_LE(std::abs(r.gate.velocity()[axis]-previous_v),cfg.acceleration[axis]*.008+1e-12);
      previous_v=r.gate.velocity()[axis];
    }
    EXPECT_GT(direction*previous_v,.02);
    EXPECT_LT(std::abs(r.gate.target()[axis]-r.q[axis]),.043);
  }
}

TEST(JointReferenceGate, GripperVelocityBiasCannotAccumulateUnboundedLead) {
  auto cfg=trajectory_goal_config();cfg.gripper_control=true;
  for(double direction:{-1.,1.}) {
    Rig r(cfg);r.follow();double previous=r.gripper_q,previous_v=0.;
    for(int k=0;k<1500;++k) {
      r.gripper_dq=direction*.0002918192918192908;r.input(true);
      r.ref.gripper_position=r.gripper_q+direction*.01;r.tick();
      ASSERT_EQ(r.gate.state(),Gate::TELEOP)<<k;
      const double v=(r.gate.gripper_target()-previous)/.008;
      EXPECT_LE(std::abs(r.gate.gripper_target()-r.gripper_q),.0033);
      EXPECT_LE(std::abs(v-previous_v),cfg.gripper_acceleration*.008+1e-12);
      previous=r.gate.gripper_target();previous_v=v;
    }
    EXPECT_NEAR(previous_v,0.,1e-10);
  }
}

TEST(JointReferenceGate, StationaryAxisWaitsInsideEnvelopeAndCanReverse) {
  for(size_t axis=0;axis<6;++axis) {
    const auto cfg=trajectory_goal_config();Rig r(cfg);
    r.q[axis]=(cfg.lower[axis]+cfg.upper[axis])*.5;r.follow();double previous_v=0.;
    for(int k=0;k<600;++k) {
      r.input(true);r.ref.positions[axis]=r.q[axis]+(k<300?.25:-.25);r.tick();
      ASSERT_EQ(r.gate.state(),Gate::TELEOP)<<axis<<" "<<k;
      const double v=r.gate.velocity()[axis];
      EXPECT_LE(std::abs(v-previous_v),cfg.acceleration[axis]*.008+1e-12);
      EXPECT_LE(std::abs(r.gate.target()[axis]-r.q[axis]),.8*cfg.tracking_error+1e-12);
      previous_v=v;
    }
    EXPECT_NEAR(r.gate.target()[axis]-r.q[axis],-.8*cfg.tracking_error,1e-9);
  }
}

TEST(JointReferenceGate, GripperGovernorWaitsAndResumesOnActualFeedback) {
  auto cfg=trajectory_goal_config();cfg.gripper_control=true;Rig r(cfg);r.follow();
  double previous=r.gripper_q,previous_v=0.,late_peak=0.;
  for(int k=0;k<750;++k) {
    const double motion=k<250?0.:(r.gate.gripper_target()-r.gripper_q)*.008/.5;
    r.gripper_q+=motion;r.gripper_dq=motion/.008;
    r.input(true);r.ref.gripper_position=std::min(0.,r.gate.gripper_target()+cfg.gripper_speed*.25);r.tick();
    ASSERT_EQ(r.gate.state(),Gate::TELEOP)<<k<<" reason="<<int(r.gate.reason());
    const double v=(r.gate.gripper_target()-previous)/.008;
    EXPECT_LE(std::abs(v-previous_v),cfg.gripper_acceleration*.008+1e-12);
    EXPECT_LE(std::abs(r.gate.gripper_target()-r.gripper_q),cfg.gripper_tracking_error);
    previous=r.gate.gripper_target();previous_v=v;if(k>250)late_peak=std::max(late_peak,v);
  }
  EXPECT_NEAR(late_peak,cfg.gripper_speed,1e-10);
}

TEST(JointReferenceGate, GovernorDoesNotHideFeedbackDisplacementFaultsOrRelaxLimits) {
  const auto cfg=trajectory_goal_config();Rig r(cfg);r.follow();
  r.q[0]-=.051;r.input(true);r.tick();
  EXPECT_EQ(r.gate.state(),Gate::STOPPING);EXPECT_EQ(r.gate.reason(),Gate::TRACKING_ERROR);
  Rig bounds(cfg);bounds.follow();bounds.input(true);bounds.ref.positions[0]=cfg.upper[0]+.001;
  bounds.tick();EXPECT_EQ(bounds.gate.state(),Gate::STOPPING);
  Rig bad(cfg);bad.follow();bad.gate.step(bad.now,.008,bad.q,bad.dq,false,nullptr,nullptr);
  EXPECT_EQ(bad.gate.state(),Gate::FAULT);
}

TEST(JointReferenceGate, ContinuousGoalsReachConfiguredSpeedOnEveryArmAxis) {
  for(size_t axis=0;axis<6;++axis) {
    const auto cfg=trajectory_goal_config();Rig r(cfg);
    r.q[axis]=(cfg.lower[axis]+cfg.upper[axis])*.5;r.follow();
    double peak=0.,previous_v=0.;
    for(int k=0;k<70;++k) {
      r.q=r.gate.target();r.dq=r.gate.velocity();r.input(true);
      // Same contract as the full Python bridge: a 0.25 s bounded future
      // goal, continually recomputed from controller output feedback.
      r.ref.positions[axis]=r.gate.target()[axis]+.25;
      r.tick();ASSERT_EQ(r.gate.state(),Gate::TELEOP)<<axis<<" "<<k;
      const double v=r.gate.velocity()[axis];
      EXPECT_LE(std::abs(v),1.+1e-12);
      EXPECT_LE(std::abs(v-previous_v),3.*.008+1e-12);
      EXPECT_LE(std::abs(r.gate.target()[axis]-r.q[axis]),.008+1e-12);
      previous_v=v;peak=std::max(peak,std::abs(v));
    }
    EXPECT_NEAR(peak,1.,1e-12)<<axis;
  }
}

TEST(JointReferenceGate, TrajectoryGoalsBrakeAtFixedTargetAndReverseWithoutVelocityJump) {
  const auto cfg=trajectory_goal_config();
  for(bool reverse:{false,true}) {
    Rig r(cfg);r.follow();double goal=r.q[0]+.5,previous_v=0.,peak=0.;
    for(int k=0;k<400;++k) {
      r.q=r.gate.target();r.dq=r.gate.velocity();r.input(true);
      if(reverse && k==65)goal=-.2;
      r.ref.positions[0]=goal;r.tick();
      ASSERT_EQ(r.gate.state(),Gate::TELEOP)<<k;
      const double v=r.gate.velocity()[0];
      EXPECT_LE(std::abs(v),1.+1e-12);
      EXPECT_LE(std::abs(v-previous_v),3.*.008+1e-12);
      EXPECT_LE(r.gate.target()[0],.7+1e-12);
      EXPECT_GE(r.gate.target()[0],-.2-1e-12);
      previous_v=v;peak=std::max(peak,std::abs(v));
    }
    EXPECT_GT(peak,.9);
    EXPECT_NEAR(r.gate.target()[0],goal,1e-10);
    EXPECT_NEAR(r.gate.velocity()[0],0.,1e-10);
  }
}

TEST(JointReferenceGate, GripperTrajectoryGoalsReachSpeedAndStopIfActuatorIsBlocked) {
  auto cfg=trajectory_goal_config(false);cfg.gripper_control=true;
  for(bool blocked:{false,true}) {
    Rig r(cfg);r.follow();double peak=0.,previous_v=0.,previous=r.gripper_q;
    for(int k=0;k<350 && r.gate.state()==Gate::TELEOP;++k) {
      if(!blocked) r.gripper_q=r.gate.gripper_target();
      r.input(true);r.ref.gripper_position=r.gate.gripper_target()+cfg.gripper_speed*.25;
      r.tick();
      if(r.gate.state()!=Gate::TELEOP) break;
      const double v=(r.gate.gripper_target()-previous)/.008;
      EXPECT_LE(std::abs(v),cfg.gripper_speed+1e-12);
      EXPECT_LE(std::abs(v-previous_v),cfg.gripper_acceleration*.008+1e-12);
      EXPECT_LE(std::abs(r.gate.gripper_target()-r.gripper_q),cfg.gripper_tracking_error);
      peak=std::max(peak,v);previous_v=v;previous=r.gate.gripper_target();
    }
    EXPECT_NEAR(peak,cfg.gripper_speed,1e-12);
    EXPECT_EQ(r.gate.state(),blocked?Gate::STOPPING:Gate::TELEOP);
    if(blocked) { EXPECT_EQ(r.gate.reason(),Gate::TRACKING_ERROR); }
  }
}

TEST(JointReferenceGate, FutureGoalsStillStopOnActualTrackingErrorAndRejectOutOfRangeGoal) {
  const auto cfg=trajectory_goal_config(false);Rig r(cfg);r.follow();
  for(int k=0;k<125 && r.gate.state()==Gate::TELEOP;++k) {
    // Actuator is blocked; a distant valid goal must not bypass output-to-
    // feedback protection or cause the output to jump to that goal.
    r.input(true);r.ref.positions[0]=r.q[0]+.4;r.tick();
    EXPECT_LE(std::abs(r.gate.target()[0]-r.q[0]),cfg.tracking_error);
  }
  EXPECT_EQ(r.gate.state(),Gate::STOPPING);EXPECT_EQ(r.gate.reason(),Gate::TRACKING_ERROR);
  Rig bounds(cfg);bounds.follow();bounds.input(true);bounds.ref.positions[0]=cfg.upper[0]+.001;
  bounds.tick();EXPECT_EQ(bounds.gate.state(),Gate::STOPPING);
}

TEST(JointReferenceGate, FastTrajectoryReleaseTimeoutAndDisarmStillStopOutput) {
  for(int stop=0;stop<3;++stop) {
    Rig r(trajectory_goal_config());r.follow();
    for(int k=0;k<60;++k) {
      r.q=r.gate.target();r.dq=r.gate.velocity();r.input(true);
      r.ref.positions[0]=r.gate.target()[0]+.25;r.tick();
      ASSERT_EQ(r.gate.state(),Gate::TELEOP);
    }
    ASSERT_NEAR(r.gate.velocity()[0],1.,1e-12);
    r.q=r.gate.target();r.dq=r.gate.velocity();
    if(stop==0) {r.input();r.tick();}
    if(stop==1) for(int k=0;k<8;++k) {r.q=r.gate.target();r.dq=r.gate.velocity();r.tick(false);}
    if(stop==2) r.command(Gate::Control::DISARM);
    EXPECT_EQ(r.gate.state(),stop==2?Gate::DISABLED:Gate::STOPPING);
    EXPECT_DOUBLE_EQ(r.gate.velocity()[0],0.);
    if(stop!=2) { EXPECT_DOUBLE_EQ(r.gate.target()[0],r.q[0]); }
  }
}
TEST(JointReferenceGate, ReleaseCapturesActualOnceAndCanRegripFromNewActual) {
  Rig r;r.follow();r.q[0]+=.003;r.input();r.tick();
  EXPECT_EQ(r.gate.state(),Gate::STOPPING);const auto stopped=r.gate.target();
  r.q[0]+=.001;
  for(int i=0;i<22;++i) {r.input();r.tick();EXPECT_EQ(r.gate.target(),stopped);}
  ASSERT_EQ(r.gate.state(),Gate::ARMED_HOLD);r.capture();EXPECT_EQ(r.gate.anchor(),r.q);
}
TEST(JointReferenceGate, RepeatedPoseCannotBeRenewedByBridgeHeartbeat) {
  Rig r;r.follow();const auto pose=r.ref.pose_monotonic_ns;const auto pose_seq=r.ref.pose_sequence;
  for(int i=0;i<24;++i) {
    r.input(true);r.ref.pose_monotonic_ns=pose;r.ref.pose_sequence=pose_seq;r.tick();
  }
  EXPECT_NE(r.gate.state(),Gate::TELEOP);
}
TEST(JointReferenceGate, BridgeDisappearanceStopsAndRequiresExplicitRearm) {
  Rig r;r.follow();for(int i=0;i<8;++i)r.tick(false);
  ASSERT_EQ(r.gate.state(),Gate::STOPPING);
  for(int i=0;i<25;++i) {r.input();r.tick();}
  ASSERT_EQ(r.gate.state(),Gate::PAUSED);
  r.input(true);r.command(Gate::Control::CAPTURE);EXPECT_EQ(r.gate.state(),Gate::PAUSED);
  r.input();r.arm();r.capture();
}

static Gate::Config auto_rearm_config() {
  auto cfg=Rig::config();cfg.auto_rearm_after_release=true;cfg.gripper_control=true;return cfg;
}
static void interrupt_input(Rig& r) {
  r.input(true);r.ref.valid=false;r.tick();
  ASSERT_EQ(r.gate.state(),Gate::STOPPING);
  for(int i=0;i<25;++i) {r.input(true);r.tick();}
  ASSERT_EQ(r.gate.state(),Gate::PAUSED);
}

TEST(JointReferenceGate, RecoveredInputRearmsOnReleaseAndRecapturesActualArmAndGripper) {
  Rig r(auto_rearm_config());r.prepare();
  for(int i=0;i<25;++i) {r.input();r.tick();}
  ASSERT_EQ(r.gate.state(),Gate::HOLD); // First ARM remains explicit.
  r.arm();r.capture();interrupt_input(r);
  for(int i=0;i<25;++i) {r.input(true);r.tick();}
  ASSERT_EQ(r.gate.state(),Gate::PAUSED); // Recovery with grip held cannot move.
  r.q[0]+=.002;r.gripper_q+=.0001;const auto origin=r.gate.session_origin();
  const auto frozen=r.gate.target();const auto frozen_gripper=r.gate.gripper_target();
  r.input();r.tick();
  ASSERT_EQ(r.gate.state(),Gate::ARMED_HOLD);EXPECT_EQ(r.gate.reason(),Gate::AUTO_REARMED);
  EXPECT_EQ(r.gate.target(),frozen);EXPECT_DOUBLE_EQ(r.gate.gripper_target(),frozen_gripper);
  r.capture();EXPECT_EQ(r.gate.anchor(),r.q);
  EXPECT_DOUBLE_EQ(r.gate.gripper_anchor(),r.gripper_q);
  EXPECT_EQ(r.gate.session_origin(),origin);
}

TEST(JointReferenceGate, RecoveryWaitsForFreshOrderedSameSessionFeedbackAndSettling) {
  for(int failure=0;failure<8;++failure) {
    Rig r(auto_rearm_config());r.follow();interrupt_input(r);
    r.input();
    if(failure==0) r.ref.valid=false;
    if(failure==1) r.ref.pose_monotonic_ns=r.now-200000000ULL;
    if(failure==2) r.ref.producer_id=99;
    if(failure==3) r.ref.session_id=99;
    if(failure==4) r.ref.clutch_epoch=0; // Out-of-order release cannot restore permission.
    if(failure==5) r.dq[0]=.04;
    if(failure==6) r.gripper_dq=.002;
    if(failure==7) r.q[0]+=.025;
    r.tick();EXPECT_EQ(r.gate.state(),Gate::PAUSED)<<failure;
  }
  Rig r(auto_rearm_config());r.follow();interrupt_input(r);r.input();
  r.gate.step(r.now,.008,r.q,r.dq,true,&r.ref,nullptr,0,false,true,r.gripper_q,r.gripper_dq);
  EXPECT_EQ(r.gate.state(),Gate::PAUSED);
}

TEST(JointReferenceGate, ReferenceTimeoutCanRecoverWithoutAnotherArmCommand) {
  Rig r(auto_rearm_config());r.follow();
  for(int i=0;i<35;++i) r.tick(false);
  ASSERT_EQ(r.gate.state(),Gate::PAUSED);
  r.input();r.tick();ASSERT_EQ(r.gate.state(),Gate::ARMED_HOLD);
  r.capture();
}

TEST(JointReferenceGate, ManualPauseDisarmFaultAndSessionChangeNeverAutoRearm) {
  for(int action=0;action<5;++action) {
    Rig r(auto_rearm_config());r.follow();
    if(action==0) r.command(Gate::Control::PAUSE);
    if(action==1) {interrupt_input(r);r.command(Gate::Control::PAUSE);}
    if(action==2) {interrupt_input(r);r.command(Gate::Control::DISARM);}
    if(action==3) {r.dq[0]=.3;r.tick();r.dq[0]=0.;}
    if(action==4) {r.input(true);r.ref.session_id=99;r.tick();}
    for(int i=0;i<35;++i) {r.input();r.tick();}
    EXPECT_EQ(r.gate.state(),action==2?Gate::DISABLED:action==3?Gate::FAULT:Gate::PAUSED)<<action;
  }
  Rig r(auto_rearm_config());r.follow();r.input(true);r.ref.valid=false;r.tick();
  // A local pause during automatic stopping revokes the pending recovery too.
  r.gate.step(r.now,.008,r.q,r.dq,true,nullptr,nullptr,Gate::Control::PAUSE);
  r.now+=8000000;
  for(int i=0;i<35;++i) {r.input();r.tick();}
  EXPECT_EQ(r.gate.state(),Gate::PAUSED);
}
TEST(JointReferenceGate, MissedClutchEdgesAndSessionChangeDoNotContinueOldTarget) {
  for(bool session_change:{false,true}) {
    Rig r;r.follow();r.input(true);
    if(session_change)r.ref.session_id=99;else r.ref.clutch_epoch+=2;
    r.tick();EXPECT_EQ(r.gate.state(),Gate::STOPPING);EXPECT_EQ(r.gate.target(),r.q);
  }
}
TEST(JointReferenceGate, OldTakeoverCannotAdvanceReference) {
  Rig r;r.follow();const auto q=r.gate.target();r.input(true);
  r.ref.takeover_id=0;r.ref.positions[0]+=.02;r.tick();EXPECT_EQ(r.gate.target(),q);
}
TEST(JointReferenceGate, InvalidNumericFutureTimeAndNonLiveInputPause) {
  for(int failure=0;failure<3;++failure) {
    Rig r;r.follow();r.input(true);
    if(failure==0)r.ref.positions[0]=std::numeric_limits<double>::quiet_NaN();
    if(failure==1)r.ref.generated_monotonic_ns=r.now+1;
    if(failure==2)r.ref.live=false;
    r.tick();EXPECT_EQ(r.gate.state(),Gate::STOPPING);
  }
}
TEST(JointReferenceGate, ActualSpeedLimitAndStaleFeedbackLatchFault) {
  Rig r;r.follow();r.dq[2]=.26;r.tick();EXPECT_EQ(r.gate.state(),Gate::FAULT);
  r.dq[2]=0;r.input();r.command(Gate::Control::PREPARE);EXPECT_EQ(r.gate.state(),Gate::FAULT);
  Rig stale;stale.follow();stale.gate.step(stale.now,.008,stale.q,stale.dq,false,nullptr,nullptr);
  EXPECT_EQ(stale.gate.state(),Gate::FAULT);
}
TEST(JointReferenceGate, LateCycleAndStopFailureLatchFault) {
  Rig late;late.follow();late.gate.step(late.now,.1,late.q,late.dq,true,nullptr,nullptr);
  EXPECT_EQ(late.gate.state(),Gate::FAULT);
  Rig stop;stop.follow();stop.input();stop.tick();stop.dq[0]=.04;
  for(int i=0;i<260;++i)stop.tick(false);
  EXPECT_EQ(stop.gate.state(),Gate::FAULT);EXPECT_EQ(stop.gate.reason(),Gate::STOP_TIMEOUT);
}
TEST(JointReferenceGate, UrgentDisarmCannotBeOverwrittenByCapture) {
  Rig r;r.follow();r.input(true);
  r.gate.step(r.now,.008,r.q,r.dq,true,&r.ref,&r.control,Gate::Control::DISARM);
  EXPECT_EQ(r.gate.state(),Gate::DISABLED);EXPECT_FALSE(r.gate.enabled());
}
TEST(JointReferenceGate, InvalidConfigurationRejected) {
  Gate::Config governor;governor.tracking_governor=true;EXPECT_FALSE(governor.valid());
  governor.trajectory_goals=true;EXPECT_TRUE(governor.valid());
  Gate::Config c;EXPECT_TRUE(c.valid());c.speed[0]=-1;EXPECT_FALSE(c.valid());
  c=Gate::Config();c.tracking_error=std::numeric_limits<double>::quiet_NaN();EXPECT_FALSE(c.valid());
  c=Gate::Config();c.zero_tolerance[2]=-.001;EXPECT_FALSE(c.valid());
  c=Gate::Config();c.zero_tolerance[5]=std::numeric_limits<double>::infinity();EXPECT_FALSE(c.valid());
}

static Gate::Config zero_tolerance_config() {
  auto cfg=Rig::config();cfg.feedback_tolerance=0.;
  cfg.zero_tolerance.fill(0.03490658503988659);return cfg;
}

TEST(JointReferenceGate, ZeroTolerancePreservesAnchorAndOnlyPermitsInwardCommands) {
  for(size_t axis:{1U,2U}) {
    Rig r(zero_tolerance_config());r.q[axis]=-.004768444342717615;
    r.follow();EXPECT_EQ(r.gate.target(),r.q);EXPECT_EQ(r.gate.anchor(),r.q);
    r.input(true);r.tick();EXPECT_EQ(r.gate.target(),r.q);
    const double old=r.q[axis];
    r.input(true);r.ref.positions[axis]=.001;r.tick();
    ASSERT_EQ(r.gate.state(),Gate::TELEOP);
    EXPECT_GT(r.gate.target()[axis],old);
    EXPECT_LE(r.gate.target()[axis]-old,r.cfg.acceleration[axis]*.008*.008+1e-12);
    // The previous tolerated coordinate cannot be commanded again after moving inward.
    r.input(true);r.ref.positions[axis]=old;r.tick();
    EXPECT_EQ(r.gate.state(),Gate::STOPPING);
    EXPECT_EQ(r.gate.target(),r.q);  // Stop still captures the actual coordinate, not zero.
  }
}

TEST(JointReferenceGate, DelayedZeroReturnReferenceHoldsInsteadOfFalseTrackingStop) {
  // Live 2026-10-08 trace: J2 starts below nominal zero. The bridge computes
  // from a 2-cycle-old reference while the C++ limiter has already moved inward.
  // That old target must not command outward travel or stop a valid stream.
  auto cfg=zero_tolerance_config();cfg.gripper_control=true;
  cfg.gripper_zero_tolerance=.0008342673824532896;
  for(bool gripper:{false,true}) {
    Rig r(cfg);r.q[1]=-.006675822079804661;r.gripper_q=.0003711988921949905;
    r.follow();
    uint64_t feedback_stamp=0;
    auto drive_inward=[&]() {
      r.input(true);
      if(feedback_stamp)r.ref.feedback_monotonic_ns=feedback_stamp;
      if(gripper)r.ref.gripper_position=0.;else r.ref.positions[1]=-.005947501819314862;
      r.tick();ASSERT_EQ(r.gate.state(),Gate::TELEOP);
    };
    drive_inward();
    const auto snapshot=r.gate.target();const double gripper_snapshot=r.gate.gripper_target();
    feedback_stamp=r.now-8000000;
    for(int i=0;i<3;++i)drive_inward();
    const auto before=r.gate.target();const auto gripper_before=r.gate.gripper_target();
    r.input(true);r.ref.feedback_monotonic_ns=feedback_stamp;
    if(gripper)r.ref.gripper_position=gripper_snapshot;else r.ref.positions[1]=snapshot[1];
    r.tick();
    ASSERT_EQ(r.gate.state(),Gate::TELEOP)<<"gripper="<<gripper<<" reason="<<r.gate.reason();
    EXPECT_GE(r.gate.target()[1],before[1]);
    EXPECT_LE(r.gate.gripper_target(),gripper_before);
    // Reusing the accepted reference while braking must not reverse back into
    // the zero tolerance zone or manufacture another tracking-error stop.
    for(int i=0;i<4;++i) {
      const auto q=r.gate.target();const auto g=r.gate.gripper_target();
      r.tick(false);ASSERT_EQ(r.gate.state(),Gate::TELEOP);
      EXPECT_GE(r.gate.target()[1],q[1]);EXPECT_LE(r.gate.gripper_target(),g);
    }
  }
}

TEST(JointReferenceGate, DiscreteBrakingReachesJointAndGripperBoundsWithoutFalseStop) {
  auto cfg=Rig::config();cfg.gripper_control=true;
  for(bool gripper:{false,true}) for(bool upper:{false,true}) {
    Rig r(cfg);
    const double goal=gripper?(upper?cfg.gripper_upper:cfg.gripper_lower):
      (upper?cfg.upper[1]:cfg.lower[1]);
    if(gripper)r.gripper_q=goal+(upper?-.001:.001);else r.q[1]=goal+(upper?-.02:.02);
    r.follow();double previous=gripper?r.gripper_q:r.q[1],velocity=0.;
    for(int i=0;i<400;++i) {
      r.input(true);
      if(gripper)r.ref.gripper_position=goal;else r.ref.positions[1]=goal;
      r.tick();
      ASSERT_EQ(r.gate.state(),Gate::TELEOP)<<"gripper="<<gripper<<" upper="<<upper<<" tick="<<i;
      const double current=gripper?r.gate.gripper_target():r.gate.target()[1];
      const double v=(current-previous)/.008;
      EXPECT_LE(std::abs(v), (gripper?cfg.gripper_speed:cfg.speed[1])+1e-10);
      EXPECT_LE(std::abs(v-velocity), (gripper?cfg.gripper_acceleration:cfg.acceleration[1])*.008+1e-10);
      if(upper)EXPECT_LE(current,goal);else EXPECT_GE(current,goal);
      previous=current;velocity=v;r.q=r.gate.target();r.gripper_q=r.gate.gripper_target();
    }
    EXPECT_NEAR(previous,goal,1e-10);
  }
}

TEST(JointReferenceGate, ZeroToleranceAcceptsBoundaryButDoesNotExtendOtherStops) {
  const auto cfg=zero_tolerance_config();
  for(size_t axis=0;axis<6;++axis) {
    for(bool lower:{false,true}) {
      const double nominal=lower?cfg.lower[axis]:cfg.upper[axis];
      const double allowed=nominal+(nominal==0. ? (lower?-1.:1.)*cfg.zero_tolerance[axis] : 0.);
      Rig at(cfg);at.q[axis]=allowed;at.prepare();
      Rig outside(cfg);outside.q[axis]=allowed+(lower?-1.:1.)*1e-9;
      outside.input();outside.command(Gate::Control::PREPARE);
      EXPECT_EQ(outside.gate.state(),Gate::DISABLED);EXPECT_FALSE(outside.gate.request_accepted());
      at.q[axis]=outside.q[axis];at.tick();
      EXPECT_EQ(at.gate.state(),Gate::FAULT);EXPECT_EQ(at.gate.reason(),Gate::ACTUAL_LIMIT);
    }
  }
}

TEST(JointReferenceGate, GripperZeroToleranceBlocksPrepareAndFaultsIfExceededWhenEnabled) {
  constexpr double tol=.0008342673824532896;
  EXPECT_TRUE(Gate::position_allowed(.0003895297016861095,-.03,0.,tol));
  EXPECT_TRUE(Gate::position_allowed(tol,-.03,0.,tol));
  EXPECT_FALSE(Gate::position_allowed(tol+1e-9,-.03,0.,tol));
  EXPECT_FALSE(Gate::position_allowed(-.04077230301060503,-.03,0.,tol));
  Rig r(zero_tolerance_config());r.input();
  r.control.controller_epoch=11;r.control.request_id=++r.request;
  r.control.generated_monotonic_ns=r.now;r.control.action=Gate::Control::PREPARE;
  r.gate.step(r.now,.008,r.q,r.dq,true,&r.ref,&r.control,0,true,false);
  EXPECT_EQ(r.gate.state(),Gate::DISABLED);EXPECT_FALSE(r.gate.request_accepted());
  r.control.request_id=++r.request;
  r.gate.step(r.now,.008,r.q,r.dq,true,&r.ref,&r.control,0,true,true);
  ASSERT_EQ(r.gate.state(),Gate::HOLD);
  r.gate.step(r.now,.008,r.q,r.dq,true,nullptr,nullptr,0,true,false);
  EXPECT_EQ(r.gate.state(),Gate::FAULT);EXPECT_EQ(r.gate.reason(),Gate::ACTUAL_LIMIT);
}

TEST(JointReferenceGate, HoldCommissioningRejectsArmAndCaptureEvenWithValidLiveInput) {
  auto cfg=Rig::config();cfg.allow_arm=false;
  cfg.session_reference_travel=.017453292519943295;cfg.session_actual_travel=.03490658503988659;
  Rig r(cfg);r.prepare();const auto held=r.gate.target();
  r.input();r.command(Gate::Control::ARM);
  EXPECT_FALSE(r.gate.request_accepted());EXPECT_FALSE(r.gate.follow_permitted());
  r.input(true);r.command(Gate::Control::CAPTURE);
  EXPECT_FALSE(r.gate.request_accepted());EXPECT_EQ(r.gate.state(),Gate::HOLD);
  r.input(true);r.ref.positions[0]+=.01;r.tick();EXPECT_EQ(r.gate.target(),held);
  r.command(Gate::Control::PAUSE);EXPECT_TRUE(r.gate.request_accepted());
  r.command(Gate::Control::DISARM);EXPECT_EQ(r.gate.state(),Gate::DISABLED);
}

TEST(JointReferenceGate, CommissioningOriginDoesNotResetOnRegripOrDisarm) {
  auto cfg=Rig::config();cfg.session_reference_travel=.017;cfg.session_actual_travel=.034;
  Rig r(cfg);r.follow();const auto origin=r.q;
  r.input();r.tick();for(int k=0;k<22;++k){r.input();r.tick();}
  r.q[0]+=.01;r.capture();EXPECT_EQ(r.gate.session_origin(),origin);
  r.input(true);r.ref.positions[0]=origin[0]+.018;r.tick();
  EXPECT_EQ(r.gate.state(),Gate::STOPPING);
  r.command(Gate::Control::DISARM);r.prepare();EXPECT_EQ(r.gate.session_origin(),origin);
  r.q[0]=origin[0]+.035;r.tick();
  EXPECT_EQ(r.gate.state(),Gate::FAULT);EXPECT_EQ(r.gate.reason(),Gate::SESSION_TRAVEL_LIMIT);
}

TEST(JointReferenceGate, CommissioningCaptureCannotExpandReferenceEnvelope) {
  auto cfg=Rig::config();cfg.session_reference_travel=.01;cfg.session_actual_travel=.03;
  Rig r(cfg);r.prepare();r.q[0]+=.015;r.input();r.command(Gate::Control::ARM);
  EXPECT_EQ(r.gate.state(),Gate::HOLD);EXPECT_FALSE(r.gate.request_accepted());
  cfg.session_actual_travel=.009;EXPECT_FALSE(cfg.valid());
  cfg.session_actual_travel=0.;EXPECT_FALSE(cfg.valid());
  cfg.session_reference_travel=-1.;EXPECT_FALSE(cfg.valid());
}

TEST(JointReferenceGate, ArmWaitsForControllerRampAndDisarmAcknowledgesUpdate) {
  Rig r;r.prepare();r.input();
  r.control.controller_epoch=11;r.control.request_id=++r.request;
  r.control.generated_monotonic_ns=r.now;r.control.action=Gate::Control::ARM;
  r.control.producer_id=33;r.control.session_id=44;
  r.gate.step(r.now,.008,r.q,r.dq,true,&r.ref,&r.control,0,false);
  EXPECT_EQ(r.gate.state(),Gate::HOLD);EXPECT_FALSE(r.gate.request_accepted());
  r.control.request_id=++r.request;r.control.action=Gate::Control::DISARM;
  r.gate.step(r.now,.008,r.q,r.dq,true,&r.ref,&r.control,Gate::Control::DISARM);
  EXPECT_EQ(r.gate.state(),Gate::DISABLED);EXPECT_EQ(r.gate.request_id(),r.control.request_id);
  EXPECT_TRUE(r.gate.request_accepted());
}

TEST(JointReferenceGate, FullGripperCapturesLimitsSpeedAndStopsWithTheArm) {
  auto cfg=Rig::config();cfg.gripper_control=true;Rig r(cfg);r.follow();
  EXPECT_DOUBLE_EQ(r.gate.gripper_anchor(),r.gripper_q);
  double before=r.gripper_q,velocity=0.;
  for(int i=0;i<150;++i) {
    r.input(true);r.ref.gripper_position=r.gripper_q+.001;r.tick();
    ASSERT_EQ(r.gate.state(),Gate::TELEOP);
    double v=(r.gate.gripper_target()-before)/.008;
    EXPECT_LE(std::abs(v),cfg.gripper_speed+1e-10);
    EXPECT_LE(std::abs(v-velocity),cfg.gripper_acceleration*.008+1e-10);
    before=r.gripper_q=r.gate.gripper_target();velocity=v;
  }
  EXPECT_GT(r.gripper_q,-.014);
  r.gripper_q+=.0001;r.input();r.tick();
  ASSERT_EQ(r.gate.state(),Gate::STOPPING);
  EXPECT_DOUBLE_EQ(r.gate.gripper_target(),r.gripper_q);
  double stopped=r.gate.gripper_target();r.gripper_q+=.0001;
  for(int i=0;i<24;++i) {r.input();r.tick();EXPECT_DOUBLE_EQ(r.gate.gripper_target(),stopped);}
  EXPECT_EQ(r.gate.state(),Gate::ARMED_HOLD);
}

TEST(JointReferenceGate, GripperInvalidLostOrStaleReferenceStopsBoth) {
  for(int failure=0;failure<4;++failure) {
    auto cfg=Rig::config();cfg.gripper_control=true;Rig r(cfg);r.follow();r.input(true);
    if(failure==0)r.ref.gripper_position=std::numeric_limits<double>::quiet_NaN();
    if(failure==1)r.ref.gripper_control=false;
    if(failure==2)r.ref.gripper_position=.001;
    if(failure==3)r.ref.takeover_id=0;
    r.tick();
    if(failure==3) {EXPECT_EQ(r.gate.state(),Gate::TELEOP);EXPECT_DOUBLE_EQ(r.gate.gripper_target(),r.gripper_q);}
    else {EXPECT_EQ(r.gate.state(),Gate::STOPPING);EXPECT_DOUBLE_EQ(r.gate.gripper_target(),r.gripper_q);}
  }
  auto cfg=Rig::config();cfg.gripper_control=true;Rig lost(cfg);lost.follow();
  for(int i=0;i<8;++i)lost.tick(false);
  EXPECT_EQ(lost.gate.state(),Gate::STOPPING);EXPECT_DOUBLE_EQ(lost.gate.gripper_target(),lost.gripper_q);
}

TEST(JointReferenceGate, GripperZeroToleranceCannotBecomeExtraClosingTravel) {
  auto cfg=Rig::config();cfg.gripper_control=true;cfg.gripper_zero_tolerance=.000834;
  Rig r(cfg);r.gripper_q=.00039;r.follow();EXPECT_DOUBLE_EQ(r.gate.gripper_target(),.00039);
  r.input(true);r.ref.gripper_position=.0005;r.tick();EXPECT_EQ(r.gate.state(),Gate::STOPPING);
  Rig fast(cfg);fast.follow();fast.gripper_dq=.011;fast.tick();EXPECT_EQ(fast.gate.state(),Gate::FAULT);
  Rig outside(cfg);outside.gripper_q=-.031;outside.input();outside.command(Gate::Control::PREPARE);
  EXPECT_EQ(outside.gate.state(),Gate::DISABLED);
}
