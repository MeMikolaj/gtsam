/**
 * @file Pose2RTV.h
 * @brief Pose2 with translational velocity
 * @author Mikolaj Kliniewski
 */

#pragma once

#include <gtsam_unstable/dllexport.h>
#include <gtsam/geometry/Pose2.h>
#include <gtsam/base/ProductLieGroup.h>

namespace gtsam {

/// Syntactic sugar to clarify components
typedef Vector2 Velocity2;

/**
 * Robot state for use with IMU measurements
 * - contains translation, translational velocity and rotation
 * TODO(frank): Alex should deprecate/move to project
 */
class GTSAM_UNSTABLE_EXPORT Pose2RTV : public ProductLieGroup<Pose2,Velocity2> {
protected:

  typedef ProductLieGroup<Pose2,Velocity2> Base;
  typedef OptionalJacobian<5, 5> ChartJacobian;

public:

  // constructors - with partial versions
  Pose2RTV() {}
  Pose2RTV(const Point2& t, const Rot2& rot, const Velocity2& vel)
  : Base(Pose2(rot, t), vel) {}
  Pose2RTV(const Rot2& rot, const Point2& t, const Velocity2& vel)
  : Base(Pose2(rot, t), vel) {}
  explicit Pose2RTV(const Point2& t)
  : Base(Pose2(Rot2(), t),Vector2::Zero()) {}
  Pose2RTV(const Pose2& pose, const Velocity2& vel)
  : Base(pose, vel) {}
  explicit Pose2RTV(const Pose2& pose)
  : Base(pose,Vector2::Zero()) {}

  // Construct from Base
  Pose2RTV(const Base& base)
  : Base(base) {}

  /** build from components - useful for data files */
  Pose2RTV(double yaw, double x, double y,
      double vx, double vy);

  /** build from single vector - useful for Matlab - in RtV format */
  explicit Pose2RTV(const Vector& v);


  // access
  const Pose2& pose() const { return first; }
  const Velocity2& v() const { return second; }
  const Point2& t() const { return pose().translation(); }
  const Rot2& R() const { return pose().rotation(); }

  // longer function names
  const Point2& translation() const { return pose().translation(); }
  const Rot2& rotation() const { return pose().rotation(); }
  const Velocity2& velocity() const { return second; }

  // Access to vector for ease of use with Matlab
  // and avoidance of Point2
  Vector vector() const;
  Vector translationVec() const { return pose().translation(); }
  const Velocity2& velocityVec() const { return velocity(); }

  // testable
  bool equals(const Pose2RTV& other, double tol=1e-6) const;
  void print(const std::string& s="") const;

  /// @name Manifold
  /// @{
  using Base::dimension;
  using Base::dim;
  using Base::Dim;
  using Base::retract;
  using Base::localCoordinates;
  using Base::LocalCoordinates;
  /// @}

  /// @name measurement functions
  /// @{

  /** range between translations */
  double range(const Pose2RTV& other,
               OptionalJacobian<1,5> H1={},
               OptionalJacobian<1,5> H2={}) const;
  /// @}

  /// @name IMU-specific
  /// @{

  /// Dynamics integrator for ground robots
  /// Always move from time 1 to time 2
  Pose2RTV planarDynamics(double vel_rate, double heading_rate, double max_accel, double dt) const;

  // /// Simulates flying robot with simple flight model
  // /// Integrates state x1 -> x2 given controls
  // /// x1 = {p1, r1, v1}, x2 = {p2, r2, v2}, all in global coordinates
  // /// @return x2
  // Pose2RTV flyingDynamics(double pitch_rate, double heading_rate, double lift_control, double dt) const;

  // /// General Dynamics update - supply control inputs in body frame
  // Pose2RTV generalDynamics(const Vector& accel, const Vector& gyro, double dt) const;

  // /// Dynamics predictor for both ground and flying robots, given states at 1 and 2
  // /// Always move from time 1 to time 2
  // /// @return imu measurement, as [accel, gyro]
  // Vector3 imuPrediction(const Pose2RTV& x2, double dt) const;

  /// predict measurement and where Point2 for x2 should be, as a way
  /// of enforcing a velocity constraint
  /// This version splits out the rotation and velocity for x2
  Point2 translationIntegration(const Rot2& r2, const Velocity2& v2, double dt) const;

  /// predict measurement and where Point2 for x2 should be, as a way
  /// of enforcing a velocity constraint
  /// This version takes a full Pose2RTV, but ignores the existing translation for x2
  inline Point2 translationIntegration(const Pose2RTV& x2, double dt) const {
    return translationIntegration(x2.rotation(), x2.velocity(), dt);
  }

  /// @return a vector for Matlab compatibility
  inline Vector translationIntegrationVec(const Pose2RTV& x2, double dt) const {
    return translationIntegration(x2, dt);
  }

  /**
   * Apply transform to this pose, with optional derivatives
   * equivalent to:
   * local = trans.transformFrom(global, Dtrans, Dglobal)
   *
   * Note: the transform jacobian convention is flipped
   */
  Pose2RTV transformed_from(const Pose2& trans,
      ChartJacobian Dglobal = {},
      OptionalJacobian<5, 3> Dtrans = {}) const;

  /// @}
  /// @name Utility functions
  /// @{

  /// RRTMbn - Function computes the rotation rate transformation matrix from
  /// body axis rates to euler angle (global) rates
  static Matrix RRTMbn(const double theta);
  static Matrix RRTMbn(const Rot2& att);

  /// RRTMnb - Function computes the rotation rate transformation matrix from
  /// euler angle rates to body axis rates
  static Matrix RRTMnb(const double theta);
  static Matrix RRTMnb(const Rot2& att);
  /// @}

private:
#if GTSAM_ENABLE_BOOST_SERIALIZATION
  /** Serialization function */
  friend class boost::serialization::access;
  template<class Archive>
  void serialize(Archive & ar, const unsigned int /*version*/) {
    ar & BOOST_SERIALIZATION_NVP(first);
    ar & BOOST_SERIALIZATION_NVP(second);
  }
#endif
};


template<>
struct traits<Pose2RTV> : public internal::LieGroup<Pose2RTV> {};

// Define Range functor specializations that are used in RangeFactor
template <typename A1, typename A2> struct Range;

template<>
struct Range<Pose2RTV, Pose2RTV> : HasRange<Pose2RTV, Pose2RTV, double> {};

} // \namespace gtsam
