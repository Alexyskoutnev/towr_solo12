#include <cmath>
#include <iostream>
#include <string>
#include <iostream>
#include <fstream>
#include <algorithm>
#include <stdexcept>

#include <towr/terrain/examples/height_map_examples.h>
#include <towr/initialization/gait_generator.h>
#include <towr/nlp_formulation.h>
#include <ifopt/ipopt_solver.h>
#include <towr/models/endeffector_mappings.h>

std::string save_file = "traj.csv";

using namespace towr;

void normalize(double vec0[], double vec1[]){
    vec1[0] = vec1[0] - vec0[0];
    vec1[1] = vec1[1] - vec0[1];
    vec1[2] = vec1[2] - vec0[2];
    vec0[0] = 0.0;
    vec0[1] = 0.0;
    vec0[2] = 0.21;
}

std::vector<std::string> split(const std::string& str, char delimiter) {
    std::vector<std::string> substrings;
    size_t start = 0;
    size_t end = str.find(delimiter);
    while (end != std::string::npos) {
        substrings.push_back(str.substr(start, end - start));
        start = end + 1;
        end = str.find(delimiter, start);
    }
    substrings.push_back(str.substr(start));
    return substrings;
}


std::string n_split_tokens(char** begin, int cnt){
  std::string input = "";
  int i = 0;
  while (*begin && i < cnt){
      std::cout << *begin << std::endl;
      if (i == 0)
        input += *begin;
      else {
        input += " ";
        input += *begin;
      }
      begin++;
      i++;
  }
  return input;
}

std::string getcmdParser(char** begin, char** end, const std::string & option, int arg_cnt){
  char** itr = std::find(begin, end, option);
  int cnt = 3;
  if (itr != end && ++itr != end){
    return n_split_tokens(itr, cnt);
  }
  return "";
}

bool cmdoptionExists(char** begin, char** end, const std::string& option){
  char** itr = std::find(begin, end, option);
  if (itr != end && ++itr != end){
    return 1;
  }
  return 0;
}

void entry(std::ofstream& file, double joints[]){
    for (int i = 0; i < 17; i++){
      file << joints[i] << ",";
    }
    file << joints[18] << "\n";
}

void getTrajectory(SplineHolder& solution, std::string save_file, double timestep){
  /*
  Collects Towr trajectory into a csv file
  */
  double t = 0.0;
  double T = solution.base_linear_->GetTotalTime();
  double csv[18];
  std::ofstream file;
  file.open(save_file);
  Eigen::VectorXd base_lin, base_ang, base_ang_vel, base_ang_acc, ee_contact, ee_motion, ee_forces;
  while (t <= T + 1e-4){
    int n_ee = solution.ee_motion_.size();
    base_lin = solution.base_linear_->GetPoint(t).p();
    base_ang = solution.base_angular_->GetPoint(t).p();
    for (int i = 0; i < 3; i++){
        csv[i] = base_lin[i];
        csv[i+3] = base_ang[i];
    }
    for (int ee_towr=0; ee_towr<n_ee; ++ee_towr) {
        ee_motion  = solution.ee_motion_.at(ee_towr)->GetPoint(t).p();
        for (int i = 0; i < 3; i++){
            csv[(ee_towr * 3) + (i + 6)] = ee_motion[i];
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
  double start_vel[3];
  double start_ang[3];
  double start_orn_vel[3];
  double goal_vel[3];
  bool _normailze;


  if (argc > 1){
    try {
    if (cmdoptionExists(argv, argv+argc, "-g"))
      {
        std::vector<std::string> cmd_return = split(getcmdParser(argv, argv+argc, "-g", 3), ' ');
        goal[0] = std::stod(cmd_return[0]);
        goal[1] = std::stod(cmd_return[1]);
        goal[2] = std::stod(cmd_return[2]);
      }
    else
      {
        goal[0] = 0.5;
        goal[1] = 0.0;
        goal[2] = 0.21;
      }
    if (cmdoptionExists(argv, argv+argc, "-s"))
      {
        std::vector<std::string> cmd_return = split(getcmdParser(argv, argv+argc, "-s", 3), ' ');
        start[0] = std::stod(cmd_return[0]);
        start[1] = std::stod(cmd_return[1]);
        start[2] = std::stod(cmd_return[2]);
      }
    else
      {
        start[0] = 0.0;
        start[1] = 0.0;
        start[2] = 0.21;
      }
    if (cmdoptionExists(argv, argv+argc, "-s_ang"))
      {
        std::vector<std::string> cmd_return = split(getcmdParser(argv, argv+argc, "-s_ang", 3), ' ');
        start_ang[0] = std::stod(cmd_return[0]);
        start_ang[1] = std::stod(cmd_return[1]);
        start_ang[2] = std::stod(cmd_return[2]);
      }
    else
      {
        start_ang[0] = 0.0;
        start_ang[1] = 0.0;
        start_ang[2] = 0.0;
      }
    if (cmdoptionExists(argv, argv+argc, "-s_vel"))
      {
        std::vector<std::string> cmd_return = split(getcmdParser(argv, argv+argc, "-s_vel", 3), ' ');
        start_vel[0] = std::stod(cmd_return[0]);
        start_vel[1] = std::stod(cmd_return[1]);
        start_vel[2] = std::stod(cmd_return[2]);
      }
    else
      {
        start_vel[0] = 0.0;
        start_vel[1] = 0.0;
        start_vel[2] = 0.0;
      }
    if (cmdoptionExists(argv, argv+argc, "-n"))
      {
        std::string cmd_return = getcmdParser(argv, argv+argc, "-n", 1);
        std::cout << "cmd return " << cmd_return << std::endl;
        if (cmd_return == "t"){
          
          _normailze = true;
        } else {
          _normailze = false;
        }
      }
    else
      {
        start_vel[0] = 0.0;
        start_vel[1] = 0.0;
        start_vel[2] = 0.0;
      }
    }
    catch (const std::invalid_argument& e){
      std::cerr << "Argument input error" << std::endl;
      std::cerr << "Error: " << e.what() << std::endl;
    }
  } else {
    start[0] = 0.0;
    start[1] = 0.0;
    start[2] = 0.21;
    start_vel[0] = 0.0;
    start_vel[1] = 0.0;
    start_vel[2] = 0.0;
    start_ang[0] = 0.0;
    start_ang[1] = 0.0;
    start_ang[2] = 0.0;
    goal[0] = 0.5;
    goal[1] = 0.0;
    goal[2] = 0.21;
  }

  //start of ground
  double z_ground = 0.0;

  //timetep 
  double _timestep = 0.01;

  //number of EE 
  int n_ee = 4;

  //trajectory run time
  // double run_time = std::abs(goal[0]) * 10;
  double run_time = 10;
  // double run_time = 5.0;

  // terrain
  formulation.terrain_ = std::make_shared<FlatGround>(0.0);
  

  // Kinematic limits and dynamic parameters of the Quadruped
  formulation.model_ = RobotModel(RobotModel::Solo12);

  //Set the initial EE position at t = 0
  auto nominal_stance_B = formulation.model_.kinematic_model_->GetNominalStanceInBase();
  formulation.initial_ee_W_ = nominal_stance_B;

  std::for_each(formulation.initial_ee_W_.begin(), formulation.initial_ee_W_.end(),
                  [&](Eigen::Vector3d& p){ p.z() = z_ground; } // feet at 0 height
  );

  // set the initial position of the quadruped
  formulation.initial_base_.lin.at(kPos).z() = -nominal_stance_B.front().z() + z_ground;
  goal[2] = formulation.initial_base_.lin.at(kPos).z();
  // start[2] = formulation.initial_base_.lin.at(kPos).z();

 

  // Starting the EE position a height zero
  for (auto ee : formulation.initial_ee_W_){
    // ee.z() = 0.0;
    std::cout << "ee start pos x -> " << ee.x() << std::endl;
    std::cout << "ee start pos y -> " << ee.y() << std::endl;
    std::cout << "ee start pos z -> " << ee.z() << std::endl;
  }

  //Normalize the start and end coords
  if (_normailze){
    // std::cout << "start" << start << std::endl;
    // std::cout << "start vals " << start[0] << ", " << start[1] << ", " << start[2] << std::endl;
    // std::cout << "goal vals " << goal[0] << ", " << goal[1] << ", " << goal[2] << std::endl;
    normalize(start, goal);
    // std::cout << "start vals " << start[0] << ", " << start[1] << ", " << start[2] << std::endl;
    // std::cout << "goal vals " << goal[0] << ", " << goal[1] << ", " << goal[2] << std::endl;
  }

  //define the start conditions for quadruped
  formulation.initial_base_.lin.at(towr::kPos).x() = start[0];
  formulation.initial_base_.lin.at(towr::kPos).y() = start[1];
  formulation.initial_base_.lin.at(towr::kPos).z() = start[2];
  formulation.initial_base_.lin.at(towr::kVel).x() = start_vel[0];
  formulation.initial_base_.lin.at(towr::kVel).y() = start_vel[1];
  formulation.initial_base_.lin.at(towr::kVel).z() = start_vel[2];
  formulation.initial_base_.ang.at(towr::kPos).x() = start_ang[0];
  formulation.initial_base_.ang.at(towr::kPos).y() = start_ang[1];
  formulation.initial_base_.ang.at(towr::kPos).z() = start_ang[2];

  // define the desired goal state of the quadruped
  formulation.final_base_.lin.at(towr::kPos) << goal[0], goal[1], goal[2];
  formulation.final_base_.lin.at(towr::kVel) << 0, 0, 0;
  formulation.final_base_.ang.at(towr::kPos) << 0, 0, 0;
  formulation.final_base_.ang.at(towr::kVel) << 0, 0, 0;

  std::cout << "base start pos x -> " << formulation.initial_base_.lin.at(kPos).x() << std::endl;
  std::cout << "base start pos y -> " << formulation.initial_base_.lin.at(kPos).y() << std::endl;
  std::cout << "base start pos z -> " << formulation.initial_base_.lin.at(kPos).z() << std::endl;
  std::cout << "base start vel x -> " << formulation.initial_base_.lin.at(kVel).x() << std::endl;
  std::cout << "base start vel y -> " << formulation.initial_base_.lin.at(kVel).y() << std::endl;
  std::cout << "base start vel z -> " << formulation.initial_base_.lin.at(kVel).z() << std::endl;
  std::cout << "base start ang x -> " << formulation.initial_base_.ang.at(kPos).x() << std::endl;
  std::cout << "base start ang y -> " << formulation.initial_base_.ang.at(kPos).y() << std::endl;
  std::cout << "base start ang z -> " << formulation.initial_base_.ang.at(kPos).z() << std::endl;

  std::cout << "base end pos x -> " << formulation.final_base_.lin.at(kPos).x() << std::endl;
  std::cout << "base end pos y -> " << formulation.final_base_.lin.at(kPos).y() << std::endl;
  std::cout << "base end pos z -> " << formulation.final_base_.lin.at(kPos).z() << std::endl;


  //auto gait generation
  auto gait_gen_ = GaitGenerator::MakeGaitGenerator(n_ee); //0 - overlap walk, 1 - fly trot, 2 - pace
  auto id_gait   = static_cast<GaitGenerator::Combos>(0);
    gait_gen_->SetCombo(id_gait);
    for (int ee=0; ee<n_ee; ++ee) {
      formulation.params_.ee_phase_durations_.push_back(gait_gen_->GetPhaseDurations(run_time, ee));
      formulation.params_.ee_in_contact_at_start_.push_back(gait_gen_->IsInContactAtStart(ee));
  }

  formulation.params_.constraints_.push_back(Parameters::BaseRom); //restricts the basemotion (adds more desicion varaibles  nd helps optumization converge)

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
  solver->SetOption("max_cpu_time", 60.0);
  solver->SetOption("print_level", 5);
  solver->SetOption("max_iter", 1000);
  solver->Solve(nlp);
  using namespace std;
  cout.precision(2);
  nlp.PrintCurrent(); // view variable-set, constraint violations, indices,...
  cout << fixed;
  getTrajectory(solution, save_file, _timestep);
}
