/**
 * @file Pose2RTT.cpp
 * @author Mikolaj Kliniewski
 */

#include <gtsam_unstable/dynamics/Pose2RTT.h>
#include <gtsam/geometry/Pose2.h>
#include <gtsam/base/Vector.h>

#include <cassert>

namespace gtsam {

using namespace std;

// static const Vector kGravity = Vector::Unit(3,2)*9.81;

/* ************************************************************************* */
// double bound(double a, double min, double max) {
//   if (a < min) return min;
//   else if (a > max) return max;
//   else return a;
// }

/* ************************************************************************* */
Pose2RTT::Pose2RTT(double yaw, double x, double y, 
    double vx, double vy, double wz) :
    Base(Pose2(Rot2(yaw), Point2(x, y)),
        Twist3(vx, vy, wz)) {
}

/* ************************************************************************* */
Pose2RTT::Pose2RTT(const Vector& rtv) :
    Base(Pose2(Rot2::fromAngle(rtv.head(1)(0)), Point2(rtv.segment(1, 2))),
        Twist3(rtv.tail(3))) {
}

/* ************************************************************************* */
Vector Pose2RTT::vector() const {
  Vector rtv(6);
  rtv.head(1) = Vector1(rotation().theta());
  rtv.segment(1,2) = translation();
  rtv.tail(3) = twist();
  return rtv;
}

/* ************************************************************************* */
bool Pose2RTT::equals(const Pose2RTT& other, double tol) const {
  return pose().equals(other.pose(), tol)
      && equal_with_abs_tol(twist(), other.twist(), tol);
}

/* ************************************************************************* */
void Pose2RTT::print(const string& s) const {
  cout << s << ":" << endl;
  gtsam::print((gtsam::Vector(1) << R().theta()).finished(), "  R:theta");
  cout << "  T" << t().transpose() << endl;
  gtsam::print((Vector)twist(), "  V_x, V_y, W_z");
}

/* ************************************************************************* */
Pose2RTT Pose2RTT::diffDriveDynamics(double lin_acc, double ang_acc, double dt) const {

  // split out initial state
  const Rot2& r1 = R();
  const Point2& t1 = t();
  const Twist3& v1 = twist();

  // Clipping
  double lin_vel2 = v1(0) + lin_acc*dt;
  double ang_vel2 = v1(2) + ang_acc*dt;


  double theta1 = r1.theta();
  double delta_theta = ang_vel2 * dt;

  Point2 delta_t;
  if (fabs(ang_vel2) > 1e-6) {
    // Circular arc integration
    double radius = lin_vel2 / ang_vel2;

    double dx = radius * (sin(theta1 + delta_theta) - sin(theta1));
    double dy = radius * (-cos(theta1 + delta_theta) + cos(theta1));

    delta_t = Point2(dx, dy);
  } else {
    // Straight line approximation
    double dx = lin_vel2 * dt * cos(theta1);
    double dy = lin_vel2 * dt * sin(theta1);
    delta_t = Point2(dx, dy);
  }

  // New pose
  Rot2 r2 = r1.retract(Vector1(delta_theta));
  Point2 t2 = t1 + delta_t;

  return Pose2RTT(r2, t2, Twist3(lin_vel2, 0.0, ang_vel2));
}

/* ************************************************************************* */
// Pose2RTT Pose2RTT::flyingDynamics(
//     double pitch_rate, double heading_rate, double lift_control, double dt) const {
//   // split out initial state
//   const Rot2& r1 = R();
//   const Twist3& v1 = v();

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
//   Twist3 v2 = v1 + Twist3(Acc_n * dt);
//   Point2 t2 = translationIntegration(r2, v2, dt);

//   return Pose2RTT(r2, t2, v2);
// }

/* ************************************************************************* */
// Pose2RTT Pose2RTT::generalDynamics(
//     const Vector& accel, const Vector& gyro, double dt) const {
//   //  Integrate Attitude Equations
//   Rot2 r2 = rotation().retract(gyro * dt);

//   //  Integrate Velocity Equations
//   Twist3 v2 = velocity() + Twist3(dt * (r2.matrix() * accel + kGravity));

//   //  Integrate Position Equations
//   Point2 t2 = translationIntegration(r2, v2, dt);

//   return Pose2RTT(t2, r2, v2);
// }

/* ************************************************************************* */
// Vector3 Pose2RTT::imuPrediction(const Pose2RTT& x2, double dt) const {
//   // split out states
//   const Rot2      &r1 = R(), &r2 = x2.R();
//   const Twist3 &v1 = v(), &v2 = x2.v();

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
// Point2 Pose2RTT::translationIntegration(const Rot2& r2, const Twist3& twist, double dt) const {
//   // predict point for constraint
//   // NOTE: uses simple Euler approach for prediction - without angular velocity

//   // Extract local linear velocity (Vx, Vy)
//   Vector2 v_local = twist.head<2>();
//   // Rotate to global frame using Rot2
//   Vector2 v_global = r2.rotate(v_local);

//   Point2 pred_t2 = t() + dt * v_global;
//   return pred_t2;
// }

/* ************************************************************************* */
// double Pose2RTT::range(const Pose2RTT& other,
//     OptionalJacobian<1,6> H1, OptionalJacobian<1,6> H2) const {
//   Matrix23 D_t1_pose, D_t2_other;
//   const Point2 t1 = pose().translation(H1 ? &D_t1_pose : 0);
//   const Point2 t2 = other.pose().translation(H2 ? &D_t2_other : 0);
//   Matrix13 D_d_t1, D_d_t2;
//   double d = distance2(t1, t2, H1 ? &D_d_t1 : 0, H2 ? &D_d_t2 : 0);
//   if (H1) {
//     H1->setZero(); // 1x6
//     H1->block<1,3>(0,0) = D_d_t1 * D_t1_pose;
//     // d/dTwist = 0
//   }
//   if (H2) {
//     H2->setZero(); // 1x6
//     H2->block<1,3>(0,0) = D_d_t2 * D_t2_other;  // d/dPose
//     // d/dTwist = 0
//   }
//   return d;
// }

/* ************************************************************************* */
Pose2RTT Pose2RTT::transformed_from(const Pose2& trans, ChartJacobian Dglobal,
    OptionalJacobian<6, 3> Dtrans) const {

  // Pose2 transform is just compose
  Matrix3 D_newpose_trans, D_newpose_pose;
  Pose2 newpose = trans.compose(pose(), D_newpose_trans, D_newpose_pose);

  // Note that we rotate the velocity
  Matrix21 D_newvel_R;
  Matrix2 D_newvel_v;
  Vector2 rotated_vel = trans.rotation().rotate(Point2(twist().head<2>()), D_newvel_R, D_newvel_v);
  
  Twist3 newtwist;
  newtwist.head<2>() = rotated_vel;
  newtwist(2) = twist()(2);

  if (Dglobal) {
    Dglobal->setZero();
    Dglobal->topLeftCorner<3,3>() = D_newpose_pose;
    Dglobal->block<2,2>(3,3) = D_newvel_v;
    (*Dglobal)(5,5) = 1.0; // dwz_new / dwz_old
  }

  if (Dtrans) {
    Dtrans->setZero();
    Dtrans->topLeftCorner<3,3>() = D_newpose_trans;
    Dtrans->block<2,1>(3,2) = D_newvel_R;
  }
  return Pose2RTT(newpose, newtwist);
}

/* ************************************************************************* */
Matrix Pose2RTT::RRTMbn(const double theta) {
  return Eigen::Matrix<double, 1, 1>::Identity();
}

/* ************************************************************************* */
Matrix Pose2RTT::RRTMbn(const Rot2& att) {
  return Pose2RTT::RRTMbn(att.theta());
}

/* ************************************************************************* */
Matrix Pose2RTT::RRTMnb(const double theta) {
  return Eigen::Matrix<double, 1, 1>::Identity();
}

/* ************************************************************************* */
Matrix Pose2RTT::RRTMnb(const Rot2& att) {
  return Pose2RTT::RRTMnb(att.theta());
}

/* ************************************************************************* */
} // \namespace gtsam
