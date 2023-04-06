/******************************************************************************
Copyright (c) 2017, Alexander W. Winkler. All rights reserved.

Redistribution and use in source and binary forms, with or without
modification, are permitted provided that the following conditions are met:

* Redistributions of source code must retain the above copyright notice, this
  list of conditions and the following disclaimer.

* Redistributions in binary form must reproduce the above copyright notice,
  this list of conditions and the following disclaimer in the documentation
  and/or other materials provided with the distribution.

* Neither the name of the copyright holder nor the names of its
  contributors may be used to endorse or promote products derived from
  this software without specific prior written permission.

THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE
FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
******************************************************************************/

#ifndef XPP_VIS_HYQLEG_INVERSE_KINEMATICS_H_
#define XPP_VIS_HYQLEG_INVERSE_KINEMATICS_H_

#include <Eigen/Dense>
#include <iostream>

using namespace Eigen;

namespace xpp {

enum HyqJointID {HAA=0, HFE, KFE, HyqlegJointCount};

/**
 * @brief Converts a hyq foot position to joint angles.
 */
class SoloInverseKinematics {
public:
  // using Vector3d = Eigen::Vector3d;
  // using Vector2d = Eigen::Vector2d;
  // using Matrix3d = Eigen::Matrix3d;
  // using Matrix2d = Eigen::Matrix2d;
  enum KneeBend { Forward, Backward };

  /**
   * @brief Default c'tor initializing leg lengths with standard values.
   */
  SoloInverseKinematics () = default;
  virtual ~SoloInverseKinematics () = default;

  /**
   * @brief Returns the joint angles to reach a Cartesian foot position.
   * @param ee_pos_H  Foot position xyz expressed in the frame attached
   * at the hip-aa (H).
   */
  Vector3d GetJointAngles(const Vector3d& ee_pos_H, bool left, KneeBend bend=Forward) const;

  /**
   * @brief Restricts the joint angles to lie inside the feasible range
   * @param q[in/out]  Current joint angle that is adapted if it exceeds
   * the specified range.
   * @param joint  Which joint (HAA, HFE, KFE) this value represents.
   */
  void EnforceLimits(double& q, HyqJointID joint) const;

  void EnforceLimitsForward(double& q, HyqJointID joint) const;

  void EnforceLimitsBackward(double& q, HyqJointID joint) const;

  Vector3d IkFunction(const Vector3d& joint_angles, const Vector3d& ee_pos_H) const;

  Matrix3d JacobIkFunction(const Vector3d& joint_angles) const;

  Vector3d Initial_approx(const Vector3d& ee_pos_B, KneeBend bend) const;

  void Test_IK(const Vector3d& ee_pos_orig, const Vector3d& joint_angles) const;

  void Test_Jacob(const Vector3d& ee_pos_H, const Vector3d& joint_angles, const Matrix3d& Jacob_cal) const;
private:
//   Vector3d hfe_to_haa_z = Vector3d(0.0, 0.014, 0.0); // distance of HFE to HAA in z direction
  Vector3d base_to_haa = Vector3d(0.1946, 0.0875, 0.0); // l1
  Vector3d haa_to_hfe = Vector3d(0.0, 0.014, 0.0); // l2'
  //Vector3d base_to_hfe = Vector3d(0.190, 0.1046, 0.0);
  Vector3d hfe_to_kfe = Vector3d(0.0, 0.03745, 0.16); // l3
  Vector3d kfe_to_foot = Vector3d(0.0, 0.008, 0.16); // l4'
  double length_thigh = 0.160; // length of upper leg
  double length_shank = 0.160; // length of lower leg

  double l1 = haa_to_hfe[1];
  double l2 = hfe_to_kfe[2];
  double l3 = hfe_to_kfe[1]+kfe_to_foot[1];
  double l4 = kfe_to_foot[2];
};

} /* namespace xpp */

#endif /* XPP_VIS_HYQLEG_INVERSE_KINEMATICS_H_ */
