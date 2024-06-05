#ifndef SO3_MATH_H
#define SO3_MATH_H

/* This code is for calculation of 3d rotation  */
#include <math.h>
#include <Eigen/Core>

/* 任何旋转向量可以表示为斜对称矩阵A的指数 */
#define SKEW_SYM_MATRX(v) 0.0, -v[2], v[1], v[2], 0.0, -v[0], -v[1], v[0], 0.0
/* Skew-symmetric matrix
  0.0 , -v[2],  v[1],
  v[2],  0.0 , -v[0],
 -v[1],  v[0],  0.0
*/

/* 3d vector to skew-symmetric matrix*/
template <typename T>
Eigen::Matrix<T, 3, 3> skew_sym_mat(const Eigen::Matrix<T, 3, 1> &v)
{
  Eigen::Matrix<T, 3, 3> skew_sym_mat;
  skew_sym_mat << 0.0, -v[2], v[1], v[2], 0.0, -v[0], -v[1], v[0], 0.0;
  return skew_sym_mat;
}

/* 3d angle to rotation matrix*/
template <typename T>
Eigen::Matrix<T, 3, 3> Exp(const Eigen::Matrix<T, 3, 1> &&ang)
{
  T ang_norm                  = ang.norm( );
  Eigen::Matrix<T, 3, 3> Eye3 = Eigen::Matrix<T, 3, 3>::Identity( );
  if (ang_norm > 0.0000001)
  {
    Eigen::Matrix<T, 3, 1> r_axis = ang / ang_norm;
    Eigen::Matrix<T, 3, 3> K;
    K << SKEW_SYM_MATRX(r_axis); // unit matrix of vector
    /// Roderigous Tranformationthe result is rotation matrix
    return Eye3 + std::sin(ang_norm) * K + (1.0 - std::cos(ang_norm)) * K * K;
  }
  else
  {
    return Eye3; // angle is 0;
  }
}

/* angular velocity + t ---> rotation matrix */
template <typename T, typename Ts>
Eigen::Matrix<T, 3, 3> Exp(const Eigen::Matrix<T, 3, 1> &ang_vel, const Ts &dt)
{
  T ang_vel_norm              = ang_vel.norm( );
  Eigen::Matrix<T, 3, 3> Eye3 = Eigen::Matrix<T, 3, 3>::Identity( );

  if (ang_vel_norm > 0.0000001) // is rotating
  {
    Eigen::Matrix<T, 3, 1> r_axis = ang_vel / ang_vel_norm;
    Eigen::Matrix<T, 3, 3> K;

    K << SKEW_SYM_MATRX(r_axis);

    T r_ang = ang_vel_norm * dt; // rotation

    /// Roderigous Tranformation
    return Eye3 + std::sin(r_ang) * K + (1.0 - std::cos(r_ang)) * K * K;
  }
  else
  {
    return Eye3;
  }
}

/* 3 value as rotation vector ---> rotation matrix*/
template <typename T>
Eigen::Matrix<T, 3, 3> Exp(const T &v1, const T &v2, const T &v3)
{
  T &&norm                    = sqrt(v1 * v1 + v2 * v2 + v3 * v3);
  Eigen::Matrix<T, 3, 3> Eye3 = Eigen::Matrix<T, 3, 3>::Identity( );
  if (norm > 0.00001)
  {
    T r_ang[3] = {v1 / norm, v2 / norm, v3 / norm};
    Eigen::Matrix<T, 3, 3> K;
    K << SKEW_SYM_MATRX(r_ang);

    /// Roderigous Tranformation
    return Eye3 + std::sin(norm) * K + (1.0 - std::cos(norm)) * K * K;
  }
  else
  {
    return Eye3;
  }
}

/* Logarithm of a Rotation Matrix; rotation matrix to skew-symmetric matrix*/
template <typename T>
Eigen::Matrix<T, 3, 1> Log(const Eigen::Matrix<T, 3, 3> &R)
{
  // trace of rotation = 1 + 2*cos(theta); theta is rotation angle
  T theta = (R.trace( ) > 3.0 - 1e-6) ? 0.0 : std::acos(0.5 * (R.trace( ) - 1));
  // K = R - R.transpose()
  Eigen::Matrix<T, 3, 1> K(R(2, 1) - R(1, 2), R(0, 2) - R(2, 0), R(1, 0) - R(0, 1));
  return (std::abs(theta) < 0.001) ? (0.5 * K) : (0.5 * theta / std::sin(theta) * K);
}

/* rotation matrix to Euler angle; */
/*
   This code is designed for drone, it's possible that pitch is close to ±90
   When this occurs, we can't calculate heading/yaw. It's almost impossible that
   roll is close to ±90, so we don't take it into consideration.
*/
template <typename T>
Eigen::Matrix<T, 3, 1> RotMtoEuler(const Eigen::Matrix<T, 3, 3> &rot)
{
  T sy          = sqrt(rot(0, 0) * rot(0, 0) + rot(1, 0) * rot(1, 0));
  bool singular = sy < 1e-6;
  T x, y, z;
  if (!singular)
  {
    x = atan2(rot(2, 1), rot(2, 2));
    y = atan2(-rot(2, 0), sy);
    z = atan2(rot(1, 0), rot(0, 0));
  }
  else
  {
    x = atan2(-rot(1, 2), rot(1, 1));
    y = atan2(-rot(2, 0), sy);
    z = 0;
  }
  Eigen::Matrix<T, 3, 1> ang(x, y, z);
  return ang;
}

#endif
