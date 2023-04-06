#ifndef XPP_HYQ_SOLO12_INVERSEKINEMATICS_H_
#define XPP_HYQ_SOLO12_INVERSEKINEMATICS_H_

#include <xpp_vis/inverse_kinematics.h>
#include <solo_inverse_kinematics.h>

namespace xpp {

/**
 * @brief Converts a hyq foot position to joint angles.
 */
class InverseKinematicsSolo12 : public InverseKinematics {
public:
  InverseKinematicsSolo12() = default;
  virtual ~InverseKinematicsSolo12() = default;

  /**
   * @brief Returns joint angles to reach for a specific foot position.
   * @param pos_B  3D-position of the foot expressed in the base frame (B).
   */
  Joints GetAllJointAngles(const EndeffectorsPos& pos_b) const override;

  /**
   * @brief Number of endeffectors (feet, hands) this implementation expects.
   */
  int GetEECount() const override { return 4; };
private:
  // Vector3d base2hip_LFKFE_ = Vector3d(0.190, 0.15005, 0.0); // check this!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!
  Vector3d base2hip_LFHAA_ = Vector3d(0.19460, 0.08750, 0.0); 
  SoloInverseKinematics leg;
};

} /* namespace xpp */

#endif /* XPP_HYQ_SOLO12_INVERSEKINEMATICS_H_ */
