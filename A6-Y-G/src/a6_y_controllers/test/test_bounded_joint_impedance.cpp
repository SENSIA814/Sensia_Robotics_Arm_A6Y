#include <a6_y_controllers/bounded_joint_impedance.hpp>
#include <gtest/gtest.h>
#include <limits>
#include <random>

using namespace a6_y_controllers;

TEST(BoundedJointImpedance, DiagonalPlantMatchesScalarSolutionWithoutCommonWristScaling)
{
  const JointMatrix mass = JointVector::LinSpaced(0.01, 0.3).asDiagonal();
  const JointVector K = JointVector::Constant(6.0), D = JointVector::Constant(0.2);
  JointVector error = JointVector::Constant(0.04), velocity = JointVector::Constant(-0.03);
  error[3] = 2.0;  // Only this wrist reaches its cap.
  const JointVector caps = JointVector::Constant(0.5);
  const auto step = bounded_joint_impedance(mass, error, velocity, K, D, 0.02, -caps, caps);
  ASSERT_TRUE(step.valid);
  for (int i = 0; i < 6; ++i)
  {
    const double A = D[i] + 0.02 * K[i];
    const double raw = (K[i] * error[i] - A * velocity[i]) / (1 + 0.02 * A / mass(i, i));
    EXPECT_NEAR(step.torque[i], std::clamp(raw, -caps[i], caps[i]), 1e-10);
    EXPECT_EQ(step.saturated[i], i == 3);
  }
}

TEST(BoundedJointImpedance, CoupledSaturatedDamperNeverAddsKineticEnergy)
{
  std::mt19937 rng(20261007);
  std::uniform_real_distribution<double> random(-1, 1);
  for (int sample = 0; sample < 600; ++sample)
  {
    JointMatrix factor;
    JointVector velocity, D, lower, upper;
    for (int i = 0; i < 6; ++i)
    {
      for (int j = 0; j < 6; ++j) factor(i, j) = random(rng);
      velocity[i] = random(rng);
      D[i] = 0.01 + 2 * std::abs(random(rng));
      lower[i] = -0.01 - 0.4 * std::abs(random(rng));
      upper[i] = 0.01 + 0.4 * std::abs(random(rng));
    }
    if (sample % 3 == 0) upper[0] = 0; // Gravity uses all positive torque room.
    if (sample % 7 == 0) lower[1] = upper[1] = 0;
    if (sample % 11 == 0) D[2] = 0;
    const JointMatrix mass = factor * factor.transpose() + 0.003 * JointMatrix::Identity();
    const double dt = sample % 2 == 0 ? 0.02 : 0.04;
    const auto step = bounded_joint_impedance(mass, JointVector::Zero(), velocity,
      JointVector::Zero(), D, dt, lower, upper);
    ASSERT_TRUE(step.valid) << sample;
    EXPECT_LE(step.torque.dot(velocity), 1e-10) << sample;
    const JointVector next = velocity + dt * mass.ldlt().solve(step.torque);
    EXPECT_LE(next.dot(mass * next), velocity.dot(mass * velocity) + 1e-9) << sample;
    const JointVector demand = -D.cwiseProduct(next);
    for (int i = 0; i < 6; ++i)
    {
      EXPECT_GE(step.torque[i], lower[i] - 1e-12);
      EXPECT_LE(step.torque[i], upper[i] + 1e-12);
      EXPECT_NEAR(step.torque[i], std::clamp(demand[i], lower[i], upper[i]), 1e-8) << sample;
    }
  }
}

TEST(BoundedJointImpedance, SpringDamperObeysBoundedImplicitLawAcrossCoupledPlants)
{
  std::mt19937 rng(4439);
  std::uniform_real_distribution<double> random(-1, 1);
  for (int sample = 0; sample < 600; ++sample)
  {
    JointMatrix factor;
    JointVector error, velocity, K, D, caps;
    for (int i = 0; i < 6; ++i)
    {
      for (int j = 0; j < 6; ++j) factor(i, j) = random(rng);
      error[i] = random(rng); velocity[i] = random(rng);
      K[i] = 20 * std::abs(random(rng)); D[i] = 2 * std::abs(random(rng));
      caps[i] = 0.001 + std::abs(random(rng));
    }
    if (sample % 9 == 0) K[0] = D[0] = 0;
    const JointMatrix mass = factor * factor.transpose() + 0.001 * JointMatrix::Identity();
    const auto step = bounded_joint_impedance(mass, error, velocity, K, D, 0.02, -caps, caps);
    ASSERT_TRUE(step.valid) << sample;
    const JointVector next = velocity + 0.02 * mass.ldlt().solve(step.torque);
    const JointVector demand = K.cwiseProduct(error - 0.02 * next) - D.cwiseProduct(next);
    for (int i = 0; i < 6; ++i)
      EXPECT_NEAR(step.torque[i], std::clamp(demand[i], -caps[i], caps[i]), 1e-7) << sample;
  }
}

TEST(BoundedJointImpedance, RejectsInvalidInputsWithoutReturningPartialTorque)
{
  for (int fault = 0; fault < 7; ++fault)
  {
    JointMatrix M = JointMatrix::Identity();
    JointVector e = JointVector::Ones(), v = JointVector::Zero();
    JointVector K = JointVector::Ones(), D = JointVector::Ones();
    JointVector lo = -JointVector::Ones(), hi = JointVector::Ones();
    double dt = 0.02;
    if (fault == 0) M(0, 0) = -1;
    if (fault == 1) M(0, 1) = 0.5;
    if (fault == 2) e[1] = std::numeric_limits<double>::quiet_NaN();
    if (fault == 3) K[0] = -1;
    if (fault == 4) lo[0] = 0.1;
    if (fault == 5) hi[0] = -0.1;
    if (fault == 6) dt = 0;
    const auto step = bounded_joint_impedance(M, e, v, K, D, dt, lo, hi);
    EXPECT_FALSE(step.valid);
    EXPECT_TRUE(step.torque.isZero());
  }
}
