#include "dog_position_control/leg_kinematics.hpp"

#include <algorithm>
#include <cmath>

#include "dog_position_control/math_utils.hpp"

namespace dog_position_control
{

// 二连杆腿的逆运动学。
IKResult inverse_kinematics(const LegGeometry & geometry, double x, double y)
{
  const double l1 = geometry.thigh_length;
  const double l2 = geometry.shank_length;

  double l_sq = x * x + y * y;
  double l = std::sqrt(l_sq);

  const double l_min = std::abs(l1 - l2) + 0.001;
  const double l_max = l1 + l2 - 0.001;
  l = std::clamp(l, l_min, l_max);
  l_sq = l * l;

  double cos_knee = (l1 * l1 + l2 * l2 - l_sq) / (2.0 * l1 * l2);
  cos_knee = std::clamp(cos_knee, -1.0, 1.0);
  const double theta_kfe = kPi - std::acos(cos_knee);

  const double phi1 = std::atan2(-x, y);
  double cos_phi2 = (l_sq + l1 * l1 - l2 * l2) / (2.0 * l * l1);
  cos_phi2 = std::clamp(cos_phi2, -1.0, 1.0);
  const double phi2 = std::acos(cos_phi2);
  const double theta_hfe = phi2 + phi1;

  return {theta_hfe, theta_kfe};
}

// 与 inverse_kinematics 对应的正运动学。
LegPose2D forward_kinematics(const LegGeometry & geometry, double hfe, double kfe)
{
  LegPose2D pose;
  pose.x =
    -geometry.thigh_length * std::sin(hfe) -
    geometry.shank_length * std::sin(hfe + kfe);
  pose.y =
    geometry.thigh_length * std::cos(hfe) +
    geometry.shank_length * std::cos(hfe + kfe);
  return pose;
}

// 把足端笛卡尔速度近似映射到关节角速度。
JointVel compute_joint_velocity(
  const LegGeometry & geometry,
  double x,
  double y,
  double vx,
  double vy)
{
  const double l1 = geometry.thigh_length;
  const double l2 = geometry.shank_length;

  double l_sq = x * x + y * y;
  double l = std::sqrt(l_sq);
  if (l < 0.01)
  {
    return {};
  }

  const double l_min = std::abs(l1 - l2) + 0.001;
  const double l_max = l1 + l2 - 0.001;
  l = std::clamp(l, l_min, l_max);
  l_sq = l * l;

  double cos_knee = (l1 * l1 + l2 * l2 - l_sq) / (2.0 * l1 * l2);
  cos_knee = std::clamp(cos_knee, -0.999, 0.999);
  double sin_knee = std::sqrt(1.0 - cos_knee * cos_knee);
  sin_knee = std::max(sin_knee, 1e-6);
  const double dtheta_kfe_dl = l / (l1 * l2 * sin_knee);

  double cos_phi2 = (l_sq + l1 * l1 - l2 * l2) / (2.0 * l * l1);
  cos_phi2 = std::clamp(cos_phi2, -0.999, 0.999);
  double sin_phi2 = std::sqrt(1.0 - cos_phi2 * cos_phi2);
  sin_phi2 = std::max(sin_phi2, 1e-6);
  const double dphi2_dl = -(l_sq - l1 * l1 + l2 * l2) / (2.0 * l_sq * l1 * sin_phi2);

  const double dl_dx = x / l;
  const double dl_dy = y / l;
  const double dphi1_dx = -y / l_sq;
  const double dphi1_dy = -x / l_sq;

  const double dtheta_hfe_dx = dphi1_dx + dphi2_dl * dl_dx;
  const double dtheta_hfe_dy = dphi1_dy + dphi2_dl * dl_dy;
  const double dtheta_kfe_dx = dtheta_kfe_dl * dl_dx;
  const double dtheta_kfe_dy = dtheta_kfe_dl * dl_dy;

  return {
    dtheta_hfe_dx * vx + dtheta_hfe_dy * vy,
    dtheta_kfe_dx * vx + dtheta_kfe_dy * vy};
}

}  // namespace dog_position_control
