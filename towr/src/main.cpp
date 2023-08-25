#include "towr/terrain/custom_terrain.hpp"
#include <algorithm>
#include <cmath>
#include <fstream>
#include <iostream>
#include <stdexcept>
#include <string>

#include <ifopt/ipopt_solver.h>
#include <towr/initialization/gait_generator.h>
#include <towr/models/endeffector_mappings.h>
#include <towr/nlp_formulation.h>
#include <towr/terrain/examples/height_map_examples.h>

std::string save_file = "traj.csv";

using namespace towr;

void
normalize(double start_CoM[], double goal_CoM[])
{
	goal_CoM[0] = goal_CoM[0] - start_CoM[0];
	goal_CoM[1] = goal_CoM[1] - start_CoM[1];
	// start_CoM[0] = 0.0;
	// start_CoM[1] = 0.0;
}

std::vector<std::string>
split(const std::string &str, char delimiter)
{
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

std::string
n_split_tokens(char **begin, int cnt)
{
	std::string input = "";
	int i = 0;
	while (*begin && i < cnt) {
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

std::string
getcmdParser(char **begin, char **end, const std::string &option, int arg_cnt)
{
	char **itr = std::find(begin, end, option);
	int cnt = 3;
	if (itr != end && ++itr != end) {
		return n_split_tokens(itr, cnt);
	}
	return "";
}

bool
cmdoptionExists(char **begin, char **end, const std::string &option)
{
	char **itr = std::find(begin, end, option);
	if (itr != end && ++itr != end) {
		return 1;
	}
	return 0;
}

void
entry(std::ofstream &file, double joints[])
{
	for (int i = 0; i < 18; i++) {
		file << joints[i] << ",";
	}
	file << joints[18] << "\n";
}

void
getTrajectory(SplineHolder &solution, std::string save_file, double timestep, double t_start)
{
	/*
	Collects Towr trajectory into a csv file
	*/
	double t;
	double T = solution.base_linear_->GetTotalTime();
	double csv[19];
	std::ofstream file;
	file.open(save_file);
	Eigen::VectorXd base_lin, base_ang, base_ang_vel, base_ang_acc, ee_contact, ee_motion,
	    ee_forces;
	while (t <= T + 1e-4) {
		int n_ee = solution.ee_motion_.size();
		base_lin = solution.base_linear_->GetPoint(t).p();
		base_ang = solution.base_angular_->GetPoint(t).p();
		csv[0] = t + t_start;
		for (int i = 0; i < 3; i++) {
			csv[i + 1] = base_lin[i];
			csv[i + 4] = base_ang[i];
		}
		for (int ee_towr = 0; ee_towr < n_ee; ++ee_towr) {
			ee_motion = solution.ee_motion_.at(ee_towr)->GetPoint(t).p();
			for (int i = 0; i < 3; i++) {
				csv[(ee_towr * 3) + (i + 7)] = ee_motion[i];
			}
		}
		entry(file, csv);
		t += timestep;
	}
	file.close();
}

int
main(int argc, char *argv[])
{
	NlpFormulation formulation;
	double goal[3];
	double start[3];
	double start_vel[3];
	double start_ang[3];
	double start_orn_vel[3];
	double goal_vel[3];
	bool _normalize = false;
	double ee1_pos[3];
	double ee2_pos[3];
	double ee3_pos[3];
	double ee4_pos[3];
	double t_start;
	std::vector<Eigen::Matrix<double, 3, 1>> EE_default;

	// Kinematic limits and dynamic parameters of the Quadruped
	formulation.model_ = RobotModel(RobotModel::Solo12);

	// Set the initial EE position at t = 0
	auto nominal_stance_B = formulation.model_.kinematic_model_->GetNominalStanceInBase();
	formulation.initial_ee_W_ = nominal_stance_B;
	EE_default = nominal_stance_B;

	if (argc > 1) {
		try {
			if (cmdoptionExists(argv, argv + argc, "-g")) {
				std::vector<std::string> cmd_return =
				    split(getcmdParser(argv, argv + argc, "-g", 3), ' ');
				goal[0] = std::stod(cmd_return[0]);
				goal[1] = std::stod(cmd_return[1]);
				goal[2] = std::stod(cmd_return[2]);
			} else {
				goal[0] = 0.5;
				goal[1] = 0.0;
				goal[2] = 0.24;
			}
			if (cmdoptionExists(argv, argv + argc, "-s")) {
				std::vector<std::string> cmd_return =
				    split(getcmdParser(argv, argv + argc, "-s", 3), ' ');
				start[0] = std::stod(cmd_return[0]);
				start[1] = std::stod(cmd_return[1]);
				start[2] = std::stod(cmd_return[2]);
			} else {
				start[0] = 0.0;
				start[1] = 0.0;
				start[2] = 0.24;
			}
			if (cmdoptionExists(argv, argv + argc, "-s_ang")) {
				std::vector<std::string> cmd_return =
				    split(getcmdParser(argv, argv + argc, "-s_ang", 3), ' ');
				start_ang[0] = std::stod(cmd_return[0]);
				start_ang[1] = std::stod(cmd_return[1]);
				start_ang[2] = std::stod(cmd_return[2]);
			} else {
				start_ang[0] = 0.0;
				start_ang[1] = 0.0;
				start_ang[2] = 0.0;
			}
			if (cmdoptionExists(argv, argv + argc, "-s_vel")) {
				std::vector<std::string> cmd_return =
				    split(getcmdParser(argv, argv + argc, "-s_vel", 3), ' ');
				start_vel[0] = std::stod(cmd_return[0]);
				start_vel[1] = std::stod(cmd_return[1]);
				start_vel[2] = std::stod(cmd_return[2]);
			} else {
				start_vel[0] = 0.0;
				start_vel[1] = 0.0;
				start_vel[2] = 0.0;
			}
			if (cmdoptionExists(argv, argv+argc, "-n"))
			  {
			    std::string cmd_return = getcmdParser(argv, argv+argc, "-n", 1);
			    std::cout << "value fo cmd_return -> " << cmd_return << std::endl;
			    std::cout << "next line" << std::endl;
			    if (cmd_return == "t"){
			      _normalize = true;
			    } else {
			      _normalize = false;
			    }
			  }
			else
			  {
			    start_vel[0] = 0.0;
			    start_vel[1] = 0.0;
			    start_vel[2] = 0.0;
			  }
			if (cmdoptionExists(argv, argv + argc, "-e1")) {
				std::vector<std::string> cmd_return =
				    split(getcmdParser(argv, argv + argc, "-e1", 3), ' ');
				ee1_pos[0] = std::stod(cmd_return[0]);
				ee1_pos[1] = std::stod(cmd_return[1]);
				ee1_pos[2] = std::stod(cmd_return[2]);
			} else {
				ee1_pos[0] = EE_default[0][0];
				ee1_pos[1] = EE_default[0][1];
				ee1_pos[2] = 0.0;
			}
			if (cmdoptionExists(argv, argv + argc, "-e2")) {
				std::vector<std::string> cmd_return =
				    split(getcmdParser(argv, argv + argc, "-e2", 3), ' ');
				ee2_pos[0] = std::stod(cmd_return[0]);
				ee2_pos[1] = std::stod(cmd_return[1]);
				ee2_pos[2] = std::stod(cmd_return[2]);
			} else {
				ee2_pos[0] = EE_default[1][0];
				ee2_pos[1] = EE_default[1][1];
				ee2_pos[2] = 0.0;
			}
			if (cmdoptionExists(argv, argv + argc, "-e3")) {
				std::vector<std::string> cmd_return =
				    split(getcmdParser(argv, argv + argc, "-e3", 3), ' ');
				ee3_pos[0] = std::stod(cmd_return[0]);
				ee3_pos[1] = std::stod(cmd_return[1]);
				ee3_pos[2] = std::stod(cmd_return[2]);
			} else {
				ee3_pos[0] = EE_default[2][0];
				ee3_pos[1] = EE_default[2][1];
				ee3_pos[2] = 0.0;
			}
			if (cmdoptionExists(argv, argv + argc, "-e4")) {
				std::vector<std::string> cmd_return =
				    split(getcmdParser(argv, argv + argc, "-e4", 3), ' ');
				ee4_pos[0] = std::stod(cmd_return[0]);
				ee4_pos[1] = std::stod(cmd_return[1]);
				ee4_pos[2] = std::stod(cmd_return[2]);
			} else {
				ee4_pos[0] = EE_default[3][0];
				ee4_pos[1] = EE_default[3][1];
				ee4_pos[2] = 0.0;
			}
			if (cmdoptionExists(argv, argv + argc, "-t")) {
				std::string cmd_return = getcmdParser(argv, argv + argc, "-t", 1);
				t_start = std::stod(cmd_return);
			} else {
				t_start = 0.0;
			}
		} catch (const std::invalid_argument &e) {
			std::cerr << "Argument input error" << std::endl;
			std::cerr << "Error: " << e.what() << std::endl;
		}
	} else {
		t_start = 0.0;
		start[0] = 0.0;
		start[1] = 0.0;
		start[2] = 0.0;
		start_vel[0] = 0.0;
		start_vel[1] = 0.0;
		start_vel[2] = 0.0;
		start_ang[0] = 0.0;
		start_ang[1] = 0.0;
		start_ang[2] = 0.0;
		goal[0] = 0.5;
		goal[1] = 0.0;
		goal[2] = 0.24;
		ee1_pos[0] = EE_default[0][0];
		ee1_pos[1] = EE_default[0][1];
		ee1_pos[2] = 0.0;
		ee2_pos[0] = EE_default[1][0];
		ee2_pos[1] = EE_default[1][1];
		ee2_pos[2] = 0.0;
		ee3_pos[0] = EE_default[2][0];
		ee3_pos[1] = EE_default[2][1];
		ee3_pos[2] = 0.0;
		ee4_pos[0] = EE_default[3][0];
		ee4_pos[1] = EE_default[3][1];
		ee4_pos[2] = 0.0;
		_normalize = false;
	}

	// end-effector vector
	std::vector<double *> EE = {ee1_pos, ee2_pos, ee3_pos, ee4_pos};

	// start of ground
	double z_ground = 0.0;

	// timetep
	double _timestep = 0.001;

	// number of EE
	int n_ee = 4;

	// trajectory run time
	double run_time = 5.0; //DONT TOUCH THIS BECAUSE MPC HYPERPARAMETERS ARE BASED ON THIS RUN TIME

	// terrain
	formulation.terrain_ = std::make_shared<CustomTerrain>("../data/heightfields/from_pybullet/towr_heightfield.txt");
	// formulation.terrain_ = std::make_shared<FlatGround>(0.0);

	int i = 0;
	std::for_each(formulation.initial_ee_W_.begin(), formulation.initial_ee_W_.end(),
	              [&](Eigen::Vector3d &p) {
			      p.x() = EE[i][0];
			      p.y() = EE[i][1];
			      p.z() = EE[i][2];
			      i++;
		      } // feet at 0 height
	);

	// Starting the EE position a height zero
	for (auto ee : formulation.initial_ee_W_) {
		std::cout << "ee start pos x -> " << ee.x() << std::endl;
		std::cout << "ee start pos y -> " << ee.y() << std::endl;
		std::cout << "ee start pos z -> " << ee.z() << std::endl;
	}

	// Normalize the start and end coords
	if (_normalize) {
		std::cout << "Utilizing a normalized coord system!" << std::endl;
		normalize(start, goal);
	}

	std::cout << "normalize value -> " << _normalize << std::endl;

	// define the start conditions for quadruped
	formulation.initial_base_.lin.at(towr::kPos).x() = start[0];
	formulation.initial_base_.lin.at(towr::kPos).y() = start[1];
	// formulation.initial_base_.lin.at(kPos).z() = -nominal_stance_B.front().z() + z_ground;
	formulation.initial_base_.lin.at(towr::kPos).z() = start[2]; // ignoring start z for now
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

	// auto gait generation
	auto gait_gen_ =
	    GaitGenerator::MakeGaitGenerator(n_ee); // 0 - overlap walk, 1 - fly trot, 2 - pace
	// auto gait_type = GaitGenerator::Custom;
	auto gait_type = GaitGenerator::C0;
	gait_gen_->SetCombo(gait_type);

	for (int ee = 0; ee < n_ee; ++ee) {
		 formulation.params_.ee_phase_durations_.push_back(gait_gen_->GetPhaseDurations(run_time, ee));
		formulation.params_.ee_in_contact_at_start_.push_back(
		    gait_gen_->IsInContactAtStart(ee));
	}
	//formulation.params_.ee_phase_durations_.push_back({.25, .5, .25});
	//formulation.params_.ee_phase_durations_.push_back({1.0});
	//formulation.params_.ee_phase_durations_.push_back({1.0});
	//formulation.params_.ee_phase_durations_.push_back({1.0});

	// formulation.params_.constraints_.push_back(Parameters::BaseRom); //restricts the
	// basemotion (adds more decision varaibles and helps optumization converge) Initialize the
	// nonlinear-programming problem with the variables, constraints and costs.
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
	solver->SetOption("max_cpu_time", 10.0);
	solver->SetOption("print_level", 5);
	solver->SetOption("max_iter", 100);
	solver->Solve(nlp);
	auto status = solver->GetReturnStatus();
	std::cout << "status -> " << status << std::endl;

	using namespace std;
	cout.precision(2);
	nlp.PrintCurrent(); // view variable-set, constraint violations, indices,...
	cout << fixed;
	getTrajectory(solution, save_file, _timestep, t_start);
	return status;
}
