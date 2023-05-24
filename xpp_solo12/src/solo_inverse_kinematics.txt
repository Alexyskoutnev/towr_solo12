/* Module containing the numerical approximation method to calculate
 * the inverse kinematics of Solo12
 * Authors: Wangzhe Sun, Praful Sigdel
 */


#include <solo_inverse_kinematics.h>

#include <cmath>
#include <map>

#include <xpp_states/cartesian_declarations.h>
#include <xpp_states/endeffector_mappings.h>

//Treats leg as front left leg

namespace xpp {

Vector3d SoloInverseKinematics::GetJointAngles(const Vector3d& ee_pos_H, bool left, KneeBend bend) const {
    Vector3d X = Initial_approx(ee_pos_H, bend);

    // std::cout << ee_pos_H << std::endl;
    // std::cout << X << std::endl;
    // if (bend==Forward) {
    //     double tmp = X[2];
    //     X[2] = -tmp;
    // } else {
    double tmp = X[1];
    X[1] = -tmp;
    // }
    // std::cout << X << std::endl;
    // std::cout << ee_pos_H << std::endl;
    
    double eps = 0.001;
    int max_it = 100;
    // std::cout << "Start iteration ..." << std::endl;
    // std::cout << "initial approximation, phi is " << X[0] << std::endl;

    int j = 1;
    for (int i = 0; (i < max_it) && (IkFunction(X, ee_pos_H).norm()>=eps); ++i) {
        Vector3d F = IkFunction(X, ee_pos_H);
        Matrix3d Jacob_F = JacobIkFunction(X);
        // Test_Jacob(ee_pos_H, X, Jacob_F);

        FullPivLU<Matrix3d> lu_decomp(Jacob_F);
        //std::cout << "Required rank is " << lu_decomp.rank() << std::endl;
        
        //assert (lu_decomp.rank() == 3);

        Vector3d dx = - (Jacob_F + 0.01 * Matrix3d::Identity()).inverse() * F;
        //std::cout << "Required dx is " << dx << std::endl;

        //Vector3d dx = Vector3d(0,0,0);

        double gamma = 1;
        Vector3d X_tmp = X + gamma * dx;
        // int j = 1;
        j = 1;

        while (IkFunction(X_tmp, ee_pos_H).norm() >= F.norm()) {
            gamma /= 2;
            X_tmp = X + gamma * dx;

            if(j >= 10)
                break;
                // break;
            
            ++j;
        }
    
        // std::cout << "Iteration " << i << ", gamma is " << gamma << ",norm is " < IkFunction(X, ee_pos_H).norm() << std::endl;
        
        X = X_tmp;
    }
    // std::cout << ee_pos_H << std::endl;

    // std::cout << X << std::endl;
    // X[2] += X[1];
    if(j >= 10) {
        if (left) {
            if (bend==Forward)
                std::cout << "Left Back Leg: ";
            else
                std::cout << "Left Front Leg: ";
        } else {
            if (bend==Forward)
                std::cout << "Right Back Leg: ";
            else
                std::cout << "Right Front Leg: ";
        }
            
        // std::cout << "optimization fails" << std::endl;
    }
    X[2] -= X[1];
    // std::cout << "after optimization, phi is " << X[0] << std::endl;
    

    // Test_IK(ee_pos_H, X);

    // std::cout << X[2] << "; " << X[1] << ", " << X[2]+X[1] << std::endl;
    // std::cout << "required X" << X << std::endl;
    //std::cout << "The required norm is" << IkFunction(X, ee_pos_H).norm() << std::endl;
     
    // if (bend==Forward)
    //     return Vector3d(X[0], X[1], -X[2]-X[1]);
    // else // backward
    //     return Vector3d(X[0], -X[1], X[2]+X[1]);


    // return Vector3d(X[0], X[1], X[2]);


    // if (bend==Forward) {
    //     double tmp = X[1];
    //     X[1] = -tmp;
    //     tmp = X[2];
    //     X[2] = -tmp;
    // }


    // // forward knee bend
    // EnforceLimits(X[0], HAA);
    // EnforceLimits(X[1], HFE);
    // EnforceLimits(X[2], KFE);




    if (!left) {
        double tmp = X[0];
        X[0] = -tmp;
    }


    if (bend==Forward)
        return Vector3d(X[0], -X[1], -X[2]);
    else // backward
        return Vector3d(X[0], X[1], X[2]);





    // if (bend==Forward)
    //     return Vector3d(X[0], X[1], X[2]);
    // else // backward
    //     return Vector3d(X[0], -X[1], -X[2]);


    // return X;
}


Vector3d SoloInverseKinematics::IkFunction(const Vector3d& joint_angles, const Vector3d& ee_pos_H) const {
    // Inverse Kinematics function for Solo12.   
    double phi = joint_angles[0];
    double psi = joint_angles[1];
    double alpha = joint_angles[2];
    Matrix2d R_phi;
    R_phi << cos(phi), sin(phi), 
            sin(phi), -cos(phi);

    // Vector3d base_to_foot = base_to_haa + haa_to_hfe + hfe_to_kfe + kfe_to_foot;
    // double x4 = base_to_foot[0];
    // double y4 = base_to_foot[1];
    // double z4 = base_to_foot[2];
    double x4 = ee_pos_H[0];
    double y4 = ee_pos_H[1];
    double z4 = ee_pos_H[2];

    // double x0 = base_to_haa[0];
    // double y0 = base_to_haa[1];
    // double z0 = base_to_haa[2]; 

    // double x0 = -haa_to_hfe[0];
    // double y0 = -haa_to_hfe[1];
    // double z0 = -haa_to_hfe[2]; 

    double x0 = 0;
    double y0 = 0;
    double z0 = 0;    

    Vector2d v = Vector2d(l1+l3, abs(l2*cos(psi))+abs(l4*cos(alpha)));
    Vector2d R_times_v = R_phi*v;

    double first = y4-y0-R_times_v[0];
    double second = z4-z0-R_times_v[1];
    double third = x4-x0-(l4*sin(alpha)+l2*sin(psi));

    return Vector3d(first, second, third);
}


Matrix3d SoloInverseKinematics::JacobIkFunction(const Vector3d& joint_angles) const {
    // Calculates the jacobian of inverse kinematics function.
    double phi = joint_angles[0];
    double psi = joint_angles[1];
    double alpha = joint_angles[2];

    double F_1_phi = (l1+l3)*sin(phi)-(l2*cos(psi)+l4*cos(alpha))*cos(phi);
    double F_2_phi = -(l1+l3)*cos(phi)-(l2*cos(psi)+l4*cos(alpha))*sin(phi);
    double F_3_phi = 0.0;

    double F_1_psi = l2*sin(psi)*sin(phi);
    double F_2_psi = -cos(phi)*l2*sin(psi);
    double F_3_psi = -l2*cos(psi);

    double F_1_alpha = sin(phi)*l4*sin(alpha);
    double F_2_alpha = -l4*cos(phi)*sin(alpha);
    double F_3_alpha = -l4*cos(alpha);

    Matrix3d result;
    result << F_1_phi, F_1_psi, F_1_alpha, 
            F_2_phi, F_2_psi, F_2_alpha, 
            F_3_phi, F_3_psi, F_3_alpha;
    return result;
}

//Treats robot as HyQ
Vector3d SoloInverseKinematics::Initial_approx(const Vector3d& ee_pos_B, KneeBend bend) const {
    
    //Initialization(Approximate) for newton's raphson method to find roots for function.

    Vector3d xr = ee_pos_B;


    double phi_bf, psi_bf, alpha_bf, theta_bf;
    double phi_br, psi_br, alpha_br, theta_br;

    phi_bf = phi_br = -atan2(xr[Y]-haa_to_hfe[1]-hfe_to_kfe[1],-xr[Z]);

    // translate into the HFE coordinate system (along Z axis)
    // xr += haa_to_hfe;  //distance of HFE to HAA

    // rotate into the HFE coordinate system (rot around X)
    Matrix3d R;
    R << 1.0, 0.0, 0.0, 
        0.0, cos(phi_bf), -sin(phi_bf), 
        0.0, sin(phi_bf), cos(phi_bf);
    xr = (R * xr).eval();

    // compute square of length from HFE to foot
    double tmp1 = pow(xr[X],2)+pow(xr[Z],2);

    // std::cout << xr[X] << " " << xr[Z] << std::endl;


    // compute temporary angles (with reachability check)
    double lu = length_thigh;  // length of upper leg
    double ll = length_shank;  // length of lower leg

    double some_random_value_for_psi = (pow(lu,2)+tmp1-pow(ll,2))/(2.*lu*sqrt(tmp1)); // this must be between -1 and 1
    if (some_random_value_for_psi > 1) {
        some_random_value_for_psi = 1;
    }
    if (some_random_value_for_psi < -1) {
        some_random_value_for_psi = -1;
    }

    // compute Hip FE angle
    double beta = acos(some_random_value_for_psi);
///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
    // if (xr[0] > 0)
    //     psi_bf = psi_br = atan2(-xr[Z],xr[X]) - 0.5*M_PI + beta;
    // else if (xr[0] < 0)
    //     psi_bf = psi_br = atan2(-xr[Z],-xr[X]) + beta;
    // else
    //     psi_bf = psi_br = beta;
    psi_bf = psi_br = abs(atan2(-xr[Z],xr[X])) - 0.5*M_PI + beta;
///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
 


    double some_random_value_for_theta = (pow(ll,2)+pow(lu,2)-tmp1)/(2.*ll*lu);
    // std::cout << some_random_value_for_theta << std::endl;
    // law of cosines give the knee angle
    if (some_random_value_for_theta > 1) {
        some_random_value_for_theta = 1;
    }
    if (some_random_value_for_theta < -1) {
        some_random_value_for_theta = -1;
    }

///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
    // double theta = M_PI - acos(some_random_value_for_theta);
    // theta_bf = theta_br = acos(some_random_value_for_theta)-M_PI;
    // std::cout << theta_bf << std::endl;
    theta_bf = theta_br = M_PI - acos(some_random_value_for_theta);
///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////



    if (bend==Forward)
        theta_bf = -theta_bf;
    else 
        psi_br = -psi_br;

    
    // forward knee bend
    // EnforceLimits(phi_bf, HAA);
    // EnforceLimits(psi_bf, HFE);
    // EnforceLimits(theta_bf, KFE);
    EnforceLimitsForward(phi_bf, HAA);
    EnforceLimitsForward(psi_bf, HFE);
    EnforceLimitsForward(theta_bf, KFE);

    // backward knee bend
    // EnforceLimits(phi_br, HAA);
    // EnforceLimits(psi_br, HFE);
    // EnforceLimits(theta_br, KFE);
    EnforceLimitsBackward(phi_br, HAA);
    EnforceLimitsBackward(psi_br, HFE);
    EnforceLimitsBackward(theta_br, KFE);

    if (bend==Forward)
        theta_bf = -theta_bf;
    else 
        psi_br = -psi_br;


    alpha_bf = theta_bf - psi_bf;
    alpha_br = theta_br - psi_br;

    //
    // if (bend==Forward)
    //     return Vector3d(phi_bf, psi_bf, theta_bf);
    // else // backward
    //     return Vector3d(phi_br, psi_br, theta_br);
    if (bend==Forward)
        return Vector3d(phi_bf, psi_bf, alpha_bf);
    else // backward
        return Vector3d(phi_br, psi_br, alpha_br);

    // if (bend==Forward)
    //     return Vector3d(phi_bf, psi_bf, theta_bf - psi_bf);
    // else // backward
    //     return Vector3d(phi_br, -psi_br, -theta_br + psi_br);

    // if (bend==Forward)
    //     return Vector3d(phi_bf, psi_bf, theta_bf);
    // else // backward
    //     return Vector3d(phi_br, -psi_br, -theta_br);
}


void
SoloInverseKinematics::EnforceLimitsForward (double& val, HyqJointID joint) const
{
  const static double haa_min = -30;
  const static double haa_max =  30;

  const static double hfe_min = 0;
  const static double hfe_max =  90;

  const static double kfe_min = -120;
  const static double kfe_max =  0;

  // reduced joint angles for optimization
  static const std::map<HyqJointID, double> max_range {
    {HAA, haa_max/180.0*M_PI},
    {HFE, hfe_max/180.0*M_PI},
    {KFE, kfe_max/180.0*M_PI}
  };

  // reduced joint angles for optimization
  static const std::map<HyqJointID, double> min_range {
    {HAA, haa_min/180.0*M_PI},
    {HFE, hfe_min/180.0*M_PI},
    {KFE, kfe_min/180.0*M_PI}
  };

  double max = max_range.at(joint);
  val = val>max? max : val;

  double min = min_range.at(joint);
  val = val<min? min : val;
}


void
SoloInverseKinematics::EnforceLimitsBackward (double& val, HyqJointID joint) const
{
  const static double haa_min = -30;
  const static double haa_max =  30;

  const static double hfe_min = -90;
  const static double hfe_max =  0;

  const static double kfe_min = 0;
  const static double kfe_max =  120;

  // reduced joint angles for optimization
  static const std::map<HyqJointID, double> max_range {
    {HAA, haa_max/180.0*M_PI},
    {HFE, hfe_max/180.0*M_PI},
    {KFE, kfe_max/180.0*M_PI}
  };

  // reduced joint angles for optimization
  static const std::map<HyqJointID, double> min_range {
    {HAA, haa_min/180.0*M_PI},
    {HFE, hfe_min/180.0*M_PI},
    {KFE, kfe_min/180.0*M_PI}
  };

  double max = max_range.at(joint);
  val = val>max? max : val;

  double min = min_range.at(joint);
  val = val<min? min : val;
}



void
SoloInverseKinematics::EnforceLimits (double& val, HyqJointID joint) const
{
  // totally exaggerated joint angle limits
//   const static double haa_min = -90;
//   const static double haa_max =  90;

//   const static double hfe_min = -90;
//   const static double hfe_max =  90;

//   const static double kfe_min = -180;
//   const static double kfe_max =  0;

  const static double haa_min = -30;
  const static double haa_max =  30;

  const static double hfe_min = -90;
  const static double hfe_max =  90;

  const static double kfe_min = -180;
  const static double kfe_max =  180;

  // reduced joint angles for optimization
  static const std::map<HyqJointID, double> max_range {
    {HAA, haa_max/180.0*M_PI},
    {HFE, hfe_max/180.0*M_PI},
    {KFE, kfe_max/180.0*M_PI}
  };

  // reduced joint angles for optimization
  static const std::map<HyqJointID, double> min_range {
    {HAA, haa_min/180.0*M_PI},
    {HFE, hfe_min/180.0*M_PI},
    {KFE, kfe_min/180.0*M_PI}
  };

  double max = max_range.at(joint);
  val = val>max? max : val;

  double min = min_range.at(joint);
  val = val<min? min : val;
}

// void SoloInverseKinematics::Test_IK(const Vector3d& ee_pos_orig, const Vector3d& joint_angles) const {

//     // Verify if the inverse kinematics function is correct.


//     double phi = joint_angles[0];
//     double psi = joint_angles[1];
//     double alpha = joint_angles[2];
//     double length_haa_to_foot_z = hfe_to_kfe[2] + kfe_to_foot[2];
//     double length_haa_to_foot_y = haa_to_hfe[1] + hfe_to_kfe[1] + kfe_to_foot[1];

//     double small_offset = length_haa_to_foot_y * sin(phi);
//     double long_vertical_line = length_haa_to_foot_z * cos(phi);
//     double ee_z_recovered = long_vertical_line - small_offset;

//     double kfe_x_recovered = hfe_to_kfe[2] * sin(psi);
//     double kfe_to_foot_x_recovered = kfe_to_foot[2] * sin(alpha);
//     double ee_x_recovered = kfe_x_recovered - kfe_to_foot_x_recovered;

    
//     double long_horizontal_line = length_haa_to_foot_y * cos(phi);
//     double longer_offset = length_haa_to_foot_z * sin(phi);
//     double ee_y_recovered = long_horizontal_line + longer_offset;

//     Vector3d ee_pos_recovered;
//     ee_pos_recovered << ee_x_recovered, ee_y_recovered, ee_z_recovered;

//     std::cout << "Real Endeffector: " << std::endl;
//     std::cout << ee_pos_orig << std::endl << std::endl;
//     std::cout << "Recovered Endeffector: " << std::endl;
//     std::cout << ee_pos_recovered << std::endl << std::endl;
// }

void SoloInverseKinematics::Test_Jacob(const Vector3d& ee_pos_H, const Vector3d& joint_angles, const Matrix3d& Jacob_cal) const {
    //Jacobian test.

    double eps = 0.00001;
    Matrix3d Jacob_approx;
    Jacob_approx.col(0) = IkFunction(Vector3d(joint_angles[0]+eps, joint_angles[1], joint_angles[2]), ee_pos_H) - IkFunction(joint_angles, ee_pos_H);
    Jacob_approx.col(1) = IkFunction(Vector3d(joint_angles[0], joint_angles[1]+eps, joint_angles[2]), ee_pos_H) - IkFunction(joint_angles, ee_pos_H);
    Jacob_approx.col(2) = IkFunction(Vector3d(joint_angles[0], joint_angles[1], joint_angles[2]+eps), ee_pos_H) - IkFunction(joint_angles, ee_pos_H);

    Jacob_approx = Jacob_approx / eps;
    
    std::cout << "Calculated Jacobian: " << std::endl;
    std::cout << Jacob_cal << std::endl << std::endl;
    std::cout << "Approximated Jacobian: " << std::endl;
    std::cout << Jacob_approx << std::endl << std::endl;
}
} /* namespace xpp */