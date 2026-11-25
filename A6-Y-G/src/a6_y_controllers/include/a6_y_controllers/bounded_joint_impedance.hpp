#pragma once

#include <Eigen/Cholesky>
#include <Eigen/Core>
#include <algorithm>
#include <array>
#include <cmath>

namespace a6_y_controllers
{
using JointVector = Eigen::Matrix<double, 6, 1>;
using JointMatrix = Eigen::Matrix<double, 6, 6>;

struct JointImpedanceStep
{
  bool valid{false};
  JointVector torque{JointVector::Zero()};
  std::array<bool, 6> saturated{};
  int iterations{0};
};

// Solve the frozen-inertia implicit joint spring/damper with torque bounds:
//   v_next = v + h M^-1 tau
//   tau = clamp(K (e - h v_next) - D v_next, lower, upper).
// With A = D + h K this is the strictly convex box QP
//   min 0.5 tau' (h M^-1 + A^-1) tau - (A^-1 K e - v)' tau.
// Zero-gain axes are fixed at zero torque. Gravity is handled by the caller;
// lower/upper must include zero, using the remaining room after gravity.
// Bounds participate in the solve, instead of clipping a coupled damper later
// or reducing every joint by the most saturated wrist's common scale.
inline JointImpedanceStep bounded_joint_impedance(
    const JointMatrix& mass, const JointVector& error, const JointVector& velocity,
    const JointVector& stiffness, const JointVector& damping, double dt,
    JointVector lower, JointVector upper)
{
  JointImpedanceStep result;
  if (!mass.allFinite() || !error.allFinite() || !velocity.allFinite() ||
      !stiffness.allFinite() || !damping.allFinite() || !lower.allFinite() || !upper.allFinite() ||
      !std::isfinite(dt) || dt <= 0 || (stiffness.array() < 0).any() ||
      (damping.array() < 0).any() || (lower.array() > 0).any() || (upper.array() < 0).any() ||
      !mass.isApprox(mass.transpose(), 1e-10)) return result;
  Eigen::LDLT<JointMatrix> inertia(mass);
  if (inertia.info() != Eigen::Success || inertia.vectorD().minCoeff() <= 0) return result;
  const JointMatrix inverse = inertia.solve(JointMatrix::Identity());
  const JointVector A = damping + dt * stiffness;
  JointMatrix H = dt * inverse;
  JointVector b = JointVector::Zero();
  std::array<int, 6> active{};  // -1 lower, 0 free, +1 upper, 2 fixed
  for (int i = 0; i < 6; ++i)
  {
    if (A[i] > 0)
    {
      H(i, i) += 1.0 / A[i];
      b[i] = (stiffness[i] / A[i]) * error[i] - velocity[i];
    }
    else
    {
      H(i, i) += 1.0;
      lower[i] = upper[i] = 0.0;
    }
    if (lower[i] == upper[i]) active[i] = 2;
  }
  if (!H.allFinite() || !b.allFinite()) return result;
  JointVector torque = JointVector::Zero();  // Feasible starting point.
  const double tolerance = 1e-10 * (1.0 + b.lpNorm<Eigen::Infinity>());
  // Six variables; bounded work in the control loop. Failure never sends a
  // partial iterate to the motors: the controller latches zero/disabled output.
  for (int iteration = 0; iteration < 96; ++iteration)
  {
    result.iterations = iteration + 1;
    const JointVector gradient = H * torque - b;
    JointMatrix reduced = H;
    JointVector rhs = -gradient;
    for (int i = 0; i < 6; ++i)
      if (active[i] != 0)
      {
        reduced.row(i).setZero();
        reduced.col(i).setZero();
        reduced(i, i) = 1.0;
        rhs[i] = 0.0;
      }
    Eigen::LDLT<JointMatrix> solver(reduced);
    if (solver.info() != Eigen::Success || solver.vectorD().minCoeff() <= 0) return result;
    const JointVector direction = solver.solve(rhs);
    if (!direction.allFinite() || !gradient.allFinite()) return result;
    if (rhs.lpNorm<Eigen::Infinity>() <= tolerance)
    {
      int release = -1;
      double violation = tolerance;
      for (int i = 0; i < 6; ++i)
      {
        const double value = active[i] == -1 ? -gradient[i] :
          (active[i] == 1 ? gradient[i] : 0.0);
        if (value > violation) { release = i; violation = value; }
      }
      if (release >= 0) { active[release] = 0; continue; }
      const JointVector demand = stiffness.cwiseProduct(error) -
        A.cwiseProduct(velocity + dt * inverse * torque);
      if (!demand.allFinite()) return result;
      result.torque = torque;
      result.valid = true;
      for (int i = 0; i < 6; ++i)
        result.saturated[i] = std::abs(torque[i] - demand[i]) > 1e-8 * (1 + std::abs(demand[i]));
      return result;
    }
    double fraction = 1.0;
    int block = -1, side = 0;
    for (int i = 0; i < 6; ++i)
      if (active[i] == 0 && direction[i] != 0.0)
      {
        const int bound_side = direction[i] > 0 ? 1 : -1;
        const double bound = bound_side == 1 ? upper[i] : lower[i];
        const double candidate = (bound - torque[i]) / direction[i];
        if (candidate < fraction)
        {
          fraction = std::max(0.0, candidate);
          block = i;
          side = bound_side;
        }
      }
    torque += fraction * direction;
    for (int i = 0; i < 6; ++i) torque[i] = std::clamp(torque[i], lower[i], upper[i]);
    if (block >= 0) active[block] = side;
  }
  return result;
}
}  // namespace a6_y_controllers
