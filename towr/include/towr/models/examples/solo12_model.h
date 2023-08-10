#ifndef TOWR_TOWR_ROS_INCLUDE_TOWR_ROS_SOLO12_MODEL_H_
#define TOWR_TOWR_ROS_INCLUDE_TOWR_ROS_SOLO12_MODEL_H_

#include <towr/models/kinematic_model.h>
#include <towr/models/single_rigid_body_dynamics.h>
#include <towr/models/endeffector_mappings.h>

namespace towr {

/**
 * @brief The Kinematics of the quadruped robot Solo12.
 */
class Solo12KinematicModel : public KinematicModel {
public:
  Solo12KinematicModel () : KinematicModel(4)
  {
    const double x_nominal_b = 0.20;
    const double y_nominal_b = 0.18;
    const double z_nominal_b = -0.24;

    // const double x_nominal_b = 0.20;
    // const double y_nominal_b = 0.18;
    // const double z_nominal_b = -0.24;

    nominal_stance_.at(LF) <<  x_nominal_b,   y_nominal_b, z_nominal_b;
    nominal_stance_.at(RF) <<  x_nominal_b,  -y_nominal_b, z_nominal_b;
    nominal_stance_.at(LH) << -x_nominal_b,   y_nominal_b, z_nominal_b;
    nominal_stance_.at(RH) << -x_nominal_b,  -y_nominal_b, z_nominal_b;

    max_dev_from_nominal_ << 0.10, 0.08, 0.075;
  }
};

/**
 * @brief The Dynamics of the quadruped robot Solo12.
 */
class Solo12DynamicModel : public SingleRigidBodyDynamics {
public:
  Solo12DynamicModel() : SingleRigidBodyDynamics(1.25123725,
                       0.00266220, -0.00003684, -0.00001716, 0.01388510, -0.00000009, 0.01605370,
                       4) {}
};

} /* namespace towr */

#endif /* TOWR_TOWR_ROS_INCLUDE_TOWR_ROS_SOLO12_MODEL_H_ */
