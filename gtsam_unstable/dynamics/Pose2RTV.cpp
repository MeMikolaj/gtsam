/**
 * @file Pose2RTV.cpp
 * @author Mikolaj Kliniewski
 */

#include <gtsam_unstable/dynamics/Pose2RTV.h>
#include <gtsam/geometry/Pose2.h>
#include <gtsam/base/Vector.h>

#include <cassert>

namespace gtsam {

using namespace std;

static const Vector kGravity = Vector::Unit(3,2)*9.81;

/* ************************************************************************* */
double bound(double a, double min, double max) {
  if (a < min) return min;
  else if (a > max) return max;
  else return a;
}

/* ************************************************************************* */
Pose2RTV::Pose2RTV(double yaw, double x, double y, 
    double vx, double vy) :
    Base(Pose2(Rot2(yaw), Point2(x, y)),
        Velocity2(vx, vy)) {
}

/* ************************************************************************* */
Pose2RTV::Pose2RTV(const Vector& rtv) :
    Base(Pose2(Rot2::fromAngle(rtv.head(1)(0)), Point2(rtv.segment(1, 2))),
        Velocity2(rtv.tail(2))) {
}

/* ************************************************************************* */
Vector Pose2RTV::vector() const {
  Vector rtv(5);
  rtv.head(1) = Vector1(rotation().theta());
  rtv.segment(1,2) = translation();
  rtv.tail(2) = velocity();
  return rtv;
}

/* ************************************************************************* */
bool Pose2RTV::equals(const Pose2RTV& other, double tol) const {
  return pose().equals(other.pose(), tol)
      && equal_with_abs_tol(velocity(), other.velocity(), tol);
}

/* ************************************************************************* */
void Pose2RTV::print(const string& s) const {
  cout << s << ":" << endl;
  gtsam::print((gtsam::Vector(1) << R().theta()).finished(), "  R:theta");
  cout << "  T" << t().transpose() << endl;
  gtsam::print((Vector)velocity(), "  V");
}

/* ************************************************************************* */
Pose2RTV Pose2RTV::planarDynamics(double vel_rate, double heading_rate,
    double max_accel, double dt) const {

  // split out initial state
  const Rot2& r1 = R();
  const Velocity2& v1 = v();

  // Update vehicle heading
  Rot2 r2 = r1.retract((Vector(1) << heading_rate * dt).finished());
  const double yaw2 = r2.theta();

  // Update vehicle position
  const double mag_v1 = v1.norm();

  // FIXME: this doesn't account for direction in velocity bounds
  double dv = bound(vel_rate - mag_v1, - (max_accel * dt), max_accel * dt);
  double mag_v2 = mag_v1 + dv;
  Velocity2 v2 = mag_v2 * Velocity2(cos(yaw2), sin(yaw2));

  Point2 t2 = translationIntegration(r2, v2, dt);

  return Pose2RTV(r2, t2, v2);
}

/* ************************************************************************* */
// Pose2RTV Pose2RTV::flyingDynamics(
//     double pitch_rate, double heading_rate, double lift_control, double dt) const {
//   // split out initial state
//   const Rot2& r1 = R();
//   const Velocity2& v1 = v();

//   // Update vehicle heading (and normalise yaw)
//   Vector rot_rates = (Vector(3) << 0.0, pitch_rate, heading_rate).finished();
//   Rot2 r2 = r1.retract(rot_rates*dt);

//   // Work out dynamics on platform
//   const double thrust = 50.0;
//   const double lift   = 50.0;
//   const double drag   = 0.1;
//   double yaw2 = r2.yaw();
//   double pitch2 = r2.pitch();
//   double forward_accel = -thrust * sin(pitch2); // r2, pitch (in global frame?) controls forward force
//   double loss_lift = lift*std::abs(sin(pitch2));
//   Rot2 yaw_correction_bn = Rot2::Yaw(yaw2);
//   Point2 forward(forward_accel, 0.0, 0.0);
//   Vector Acc_n =
//       yaw_correction_bn.rotate(forward)              // applies locally forward force in the global frame
//       - drag * (Vector(3) << v1.x(), v1.y(), 0.0).finished()  // drag term dependent on v1
//       + Vector::Unit(3,2)*(loss_lift - lift_control);                // falling due to lift lost from pitch

//   // Update Vehicle Position and Velocity
//   Velocity2 v2 = v1 + Velocity2(Acc_n * dt);
//   Point2 t2 = translationIntegration(r2, v2, dt);

//   return Pose2RTV(r2, t2, v2);
// }

/* ************************************************************************* */
// Pose2RTV Pose2RTV::generalDynamics(
//     const Vector& accel, const Vector& gyro, double dt) const {
//   //  Integrate Attitude Equations
//   Rot2 r2 = rotation().retract(gyro * dt);

//   //  Integrate Velocity Equations
//   Velocity2 v2 = velocity() + Velocity2(dt * (r2.matrix() * accel + kGravity));

//   //  Integrate Position Equations
//   Point2 t2 = translationIntegration(r2, v2, dt);

//   return Pose2RTV(t2, r2, v2);
// }

/* ************************************************************************* */
// Vector3 Pose2RTV::imuPrediction(const Pose2RTV& x2, double dt) const {
//   // split out states
//   const Rot2      &r1 = R(), &r2 = x2.R();
//   const Velocity2 &v1 = v(), &v2 = x2.v();

//   Vector3 imu;

//   // acceleration
//   Vector2 accel = (v2-v1) / dt;
//   imu.head<3>() = r2.transpose() * (accel - kGravity);

//   // rotation rates
//   // just using euler angles based on matlab code
//   // FIXME: this is silly - we shouldn't use differences in Euler angles
//   Matrix Enb = RRTMnb(r1);
//   Vector3 euler1 = r1.xyz(), euler2 = r2.xyz();
//   Vector3 dR = euler2 - euler1;

//   // normalize yaw in difference (as per Mitch's code)
//   dR(2) = Rot2::fromAngle(dR(2)).theta();
//   dR /= dt;
//   imu.tail<3>() = Enb * dR;
// //  imu.tail(3) = r1.transpose() * dR;

//   return imu;
// }

/* ************************************************************************* */
Point2 Pose2RTV::translationIntegration(const Rot2& r2, const Velocity2& v2, double dt) const {
  // predict point for constraint
  // NOTE: uses simple Euler approach for prediction
  Point2 pred_t2 = t() + Point2(v2 * dt);
  return pred_t2;
}

/* ************************************************************************* */
double Pose2RTV::range(const Pose2RTV& other,
    OptionalJacobian<1,5> H1, OptionalJacobian<1,5> H2) const {
  Matrix23 D_t1_pose, D_t2_other;
  const Point2 t1 = pose().translation(H1 ? &D_t1_pose : 0);
  const Point2 t2 = other.pose().translation(H2 ? &D_t2_other : 0);
  Matrix12 D_d_t1, D_d_t2;
  double d = distance2(t1, t2, H1 ? &D_d_t1 : 0, H2 ? &D_d_t2 : 0);
  if (H1) *H1 << D_d_t1 * D_t1_pose, 0,0;
  if (H2) *H2 << D_d_t2 * D_t2_other, 0,0;
  return d;
}

/* ************************************************************************* */
Pose2RTV Pose2RTV::transformed_from(const Pose2& trans, ChartJacobian Dglobal,
    OptionalJacobian<5, 3> Dtrans) const {

  // Pose2 transform is just compose
  Matrix3 D_newpose_trans, D_newpose_pose;
  Pose2 newpose = trans.compose(pose(), D_newpose_trans, D_newpose_pose);

  // Note that we rotate the velocity
  Matrix21 D_newvel_R;
  Matrix2 D_newvel_v;
  Velocity2 newvel = trans.rotation().rotate(Point2(velocity()), D_newvel_R, D_newvel_v);

  if (Dglobal) {
    Dglobal->setZero();
    Dglobal->topLeftCorner<3,3>() = D_newpose_pose;
    Dglobal->bottomRightCorner<2,2>() = D_newvel_v;
  }

  if (Dtrans) {
    Dtrans->setZero();
    Dtrans->topLeftCorner<3,3>() = D_newpose_trans;
    Dtrans->block<2,1>(3,2) = D_newvel_R;
  }
  return Pose2RTV(newpose, newvel);
}

/* ************************************************************************* */
Matrix Pose2RTV::RRTMbn(const double theta) {
  return Eigen::Matrix<double, 1, 1>::Identity();
}

/* ************************************************************************* */
Matrix Pose2RTV::RRTMbn(const Rot2& att) {
  return Pose2RTV::RRTMbn(att.theta());
}

/* ************************************************************************* */
Matrix Pose2RTV::RRTMnb(const double theta) {
  return Eigen::Matrix<double, 1, 1>::Identity();
}

/* ************************************************************************* */
Matrix Pose2RTV::RRTMnb(const Rot2& att) {
  return Pose2RTV::RRTMnb(att.theta());
}

/* ************************************************************************* */
} // \namespace gtsam
