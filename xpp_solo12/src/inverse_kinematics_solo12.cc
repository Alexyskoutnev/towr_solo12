/* 
Program contains the inverse kinematics for the SOLO12 robot. 

*/

#include <inverse_kinematics_solo12.h>

#include <cmath>
#include <map>

#include <xpp_states/cartesian_declarations.h>
#include <xpp_states/endeffector_mappings.h>


namespace xpp {

Joints
InverseKinematicsSolo12::GetAllJointAngles(const EndeffectorsPos& x_B) const
{
  Vector3d ee_pos_H; // foothold expressed in hip frame
  std::vector<Eigen::VectorXd> q_vec;

  // make sure always exactly 4 elements
  auto pos_B = x_B.ToImpl();
  pos_B.resize(4, pos_B.front());
  bool left = true;

  for (int ee=0; ee<pos_B.size(); ++ee) {

    SoloInverseKinematics::KneeBend bend = SoloInverseKinematics::Forward;
    using namespace quad;
    switch (ee) {
      case LF:
        // std::cout << "LF" << std::endl;
        // std::cout << pos_B.at(ee) << std::endl;
        ee_pos_H = pos_B.at(ee);
        left = true;
        break;
      case RF:
        // std::cout << "RF" << std::endl;
        ee_pos_H = pos_B.at(ee).cwiseProduct(Eigen::Vector3d(1,-1,1));
        left = false;
        break;
      case LH:
        // std::cout << "LH" << std::endl;
        // std::cout << pos_B.at(ee) << std::endl;
        ee_pos_H = pos_B.at(ee).cwiseProduct(Eigen::Vector3d(-1,1,1));
        // std::cout << ee_pos_H << std::endl;
        bend = SoloInverseKinematics::Backward;
        left = true;
        break;
      case RH:
        // std::cout << "RH" << std::endl;
        ee_pos_H = pos_B.at(ee).cwiseProduct(Eigen::Vector3d(-1,-1,1));
        bend = SoloInverseKinematics::Backward;
        left = false;
        break;
      default: // joint angles for this foot do not exist
        break;
    }

    ee_pos_H -= base2hip_LFHAA_; // Coordinate system translated from base to HAA at x_hfe and y_haa.
    q_vec.push_back(leg.GetJointAngles(ee_pos_H, left, bend));
  }

  return Joints(q_vec);
}

} /* namespace xpp */
