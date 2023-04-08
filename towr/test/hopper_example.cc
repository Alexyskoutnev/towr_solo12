/******************************************************************************
Copyright (c) 2018, Alexander W. Winkler. All rights reserved.

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

#include <cmath>
#include <iostream>
#include <string>
#include <iostream>
#include <fstream>

#include <towr/terrain/examples/height_map_examples.h>
#include <towr/initialization/gait_generator.h>
#include <towr/nlp_formulation.h>
#include <ifopt/ipopt_solver.h>

#include <towr/models/endeffector_mappings.h>

std::string save_file = "traj.csv";

using namespace towr;


void entry(std::ofstream& file, double joints[]){
    for (int i = 0; i < 14; i++){
      file << joints[i] << ",";
    }
    file << joints[15] << "\n";
}

void getTrajectory(SplineHolder& solution, std::string save_file, double timestep){
  double t = 0.0;
  double T = solution.base_linear_->GetTotalTime();
  double csv[15];
  std::ofstream file;
  file.open(save_file);
  Eigen::VectorXd base_lin, base_ang, base_ang_vel, base_ang_acc, ee_contact, ee_motion, ee_forces;
  while (t <= T + 1e-4){
    int n_ee = solution.ee_motion_.size();
    base_lin = solution.base_linear_->GetPoint(t).p();
    for (int i = 0; i < 3; i++){
        csv[i] = base_lin[i];
    }
    for (int ee_towr=0; ee_towr<n_ee; ++ee_towr) {
        ee_motion  = solution.ee_motion_.at(ee_towr)->GetPoint(t).p();
        // std::cout << "EE-POS [" << ee_towr << "] " << std::endl;
        // std::cout << ee_motion << std::endl;
        for (int i = 0; i < 3; i++){
            csv[(ee_towr * 3) + (i + 3)] = ee_motion[i];
        }
    }
    entry(file, csv);
    t += timestep;
  }
  file.close();
}


int main(int argc, char* argv[])
{
  NlpFormulation formulation;
  double goal[3];
  double start[3];
  //start-end position
  if (argc > 1){
    std::cout << "Using CMD VALUES" << std::endl;
    for (int i = 1; i < argc; i++){
      goal[i - 1] = std::stod(argv[i]);
    }
  } else {
      std::cout << "Default" << std::endl;
      goal[0] = 1.0;
      goal[1] = 0.0;
      goal[2] = 0.317;
  }

  //timetep 
  double timestep = 0.01;

  //number of EE 
  int n_ee = 4;

  //trajectory run time
  double run_time = 10.0;

  // terrain
  formulation.terrain_ = std::make_shared<FlatGround>(0.0);

  // Kinematic limits and dynamic parameters of the Quadruped
  // formulation.model_ = RobotModel(RobotModel::Biped);
  formulation.model_ = RobotModel(RobotModel::Solo12);
  // formulation.model_ = RobotModel(RobotModel::Anymal);

  //Set the initial EE position at t = 0
  auto nominal_stance_B = formulation.model_.kinematic_model_->GetNominalStanceInBase();
  formulation.initial_ee_W_ = nominal_stance_B;

  // set the initial position of the quadruped
  formulation.initial_base_.lin.at(kPos).z() = 0.317;
  
  // define the desired goal state of the quadruped
  formulation.final_base_.lin.at(towr::kPos) << goal[0], goal[1], goal[2];

  // Parameters that define the motion. See c'tor for default values or
  // other values that can be modified.
  // First we define the initial phase durations, that can however be changed
  // by the optimizer. The number of swing and stance phases however is fixed.
  // alternating stance and swing:     ____-----_____-----_____-----_____
  auto gait_gen_ = GaitGenerator::MakeGaitGenerator(n_ee);
  auto id_gait   = static_cast<GaitGenerator::Combos>(1);
    gait_gen_->SetCombo(id_gait);
    for (int ee=0; ee<n_ee; ++ee) {
      formulation.params_.ee_phase_durations_.push_back(gait_gen_->GetPhaseDurations(run_time, ee));
      formulation.params_.ee_in_contact_at_start_.push_back(gait_gen_->IsInContactAtStart(ee));
  }

  // Initialize the nonlinear-programming problem with the variables,
  // constraints and costs.
  ifopt::Problem nlp;
  SplineHolder solution;
  for (auto c : formulation.GetVariableSets(solution))
    nlp.AddVariableSet(c);
  for (auto c : formulation.GetConstraints(solution))
    nlp.AddConstraintSet(c);
  for (auto c : formulation.GetCosts())
    nlp.AddCostSet(c);

  // You can add your own elements to the nlp as well, simply by calling:
  // nlp.AddVariablesSet(your_custom_variables);
  // nlp.AddConstraintSet(your_custom_constraints);

  // Choose ifopt solver (IPOPT or SNOPT), set some parameters and solve.
  // solver->SetOption("derivative_test", "first-order");
  auto solver = std::make_shared<ifopt::IpoptSolver>();
  solver->SetOption("jacobian_approximation", "exact"); // "finite difference-values"
  solver->SetOption("max_cpu_time", 30.0);
  solver->Solve(nlp);

  // Can directly view the optimization variables through:
  // Eigen::VectorXd x = nlp.GetVariableValues()
  // However, it's more convenient to access the splines constructed from these
  // variables and query their values at specific times:
  using namespace std;
  cout.precision(2);
  nlp.PrintCurrent(); // view variable-set, constraint violations, indices,...
  cout << fixed;
  cout << "\n====================\nQuadruped trajectory:\n====================\n";

  double t = 0.0;
  while (t<=solution.base_linear_->GetTotalTime() + 1e-5) {
    cout << "t=" << t << "\n";
    cout << "Base linear position x,y,z:   \t";
    cout << solution.base_linear_->GetPoint(t).p().transpose() << "\t[m]" << endl;

    cout << "Base Euler roll, pitch, yaw:  \t";
    Eigen::Vector3d rad = solution.base_angular_->GetPoint(t).p();
    cout << (rad/M_PI*180).transpose() << "\t[deg]" << endl;


    for (int i = 0; i < n_ee; i ++){
       cout << "Foot position " << i << " x,y,z:          \t";
      cout << solution.ee_motion_.at(i)->GetPoint(t).p().transpose() << "\t[m]" << endl;

      cout << "Contact force " << i << " x,y,z:          \t";
      cout << solution.ee_force_.at(i)->GetPoint(t).p().transpose() << "\t[N]" << endl;

      bool contact = solution.phase_durations_.at(i)->IsContactPhase(t);
      std::string foot_in_contact = contact? "yes" : "no";
      cout << "Foot " << i << " in contact:              \t" + foot_in_contact << endl;

    }
    cout << endl;
    t += 0.1;
  }
  getTrajectory(solution, save_file, timestep);
}
