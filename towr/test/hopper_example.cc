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
  /*
  Collects Towr trajectory into a csv file
  */
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
    for (int i = 1; i < argc - 1; i++){
      goal[i - 1] = std::stod(argv[i]);
    }
  } else {
      std::cout << "Default" << std::endl;
      goal[0] = 0.5;
      goal[1] = 0.0;
  }

  //start of ground
  double z_ground = 0.0;

  //timetep 
  double _timestep = 0.01;

  //number of EE 
  int n_ee = 4;

  //trajectory run time
  double run_time = 5.0;

  // terrain
  formulation.terrain_ = std::make_shared<FlatGround>(0.0);
  

  // Kinematic limits and dynamic parameters of the Quadruped
  formulation.model_ = RobotModel(RobotModel::Solo12);

  //Set the initial EE position at t = 0
  auto nominal_stance_B = formulation.model_.kinematic_model_->GetNominalStanceInBase();
  formulation.initial_ee_W_ = nominal_stance_B;

  // set the initial position of the quadruped
  formulation.initial_base_.lin.at(kPos).z() = 0.21;
  // formulation.initial_base_.lin.at(kPos).z() = - nominal_stance_B.front().z() + z_ground;
  goal[2] = formulation.initial_base_.lin.at(kPos).z();

  std::cout << "base start pos x -> " << formulation.initial_base_.lin.at(kPos).x() << std::endl;
  std::cout << "base start pos y -> " << formulation.initial_base_.lin.at(kPos).y() << std::endl;
  std::cout << "base start pos z -> " << formulation.initial_base_.lin.at(kPos).z() << std::endl;

  // Starting the EE position a height zero
  for (auto ee : formulation.initial_ee_W_){
    // ee.z() = 0.0;
    std::cout << "ee start pos x -> " << ee.x() << std::endl;
    std::cout << "ee start pos y -> " << ee.y() << std::endl;
    std::cout << "ee start pos z -> " << ee.z() << std::endl;
  }
  
  // define the desired goal state of the quadruped
  formulation.final_base_.lin.at(towr::kPos) << goal[0], goal[1], goal[2];
  formulation.final_base_.lin.at(towr::kVel) << 0, 0, 0;
  formulation.final_base_.ang.at(towr::kPos) << 0, 0, 0;
  formulation.final_base_.ang.at(towr::kVel) << 0.5, 0, 0;


  //auto gait generation
  auto gait_gen_ = GaitGenerator::MakeGaitGenerator(n_ee); //0 - overlap walk, 1 - fly trot, 2 - pace
  auto id_gait   = static_cast<GaitGenerator::Combos>(0);
    gait_gen_->SetCombo(id_gait);
    for (int ee=0; ee<n_ee; ++ee) {
      formulation.params_.ee_phase_durations_.push_back(gait_gen_->GetPhaseDurations(run_time, ee));
      formulation.params_.ee_in_contact_at_start_.push_back(gait_gen_->IsInContactAtStart(ee));
  }

  formulation.params_.constraints_.push_back(Parameters::BaseRom); //restricts the basemotion (adds more desicion varaibles  nd helps optumization converge)
  formulation.params_.OptimizePhaseDurations();

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

  // Choose ifopt solver (IPOPT or SNOPT), set some parameters and solve.
  // solver->SetOption("derivative_test", "first-order");
  auto solver = std::make_shared<ifopt::IpoptSolver>();
  solver->SetOption("linear_solver", "mumps");
  solver->SetOption("jacobian_approximation", "exact"); // "finite difference-values"
  solver->SetOption("max_cpu_time", 30.0);
  solver->SetOption("print_level", 5);
  solver->Solve(nlp);
  using namespace std;
  cout.precision(2);
  nlp.PrintCurrent(); // view variable-set, constraint violations, indices,...
  cout << fixed;
  getTrajectory(solution, save_file, _timestep);
}
