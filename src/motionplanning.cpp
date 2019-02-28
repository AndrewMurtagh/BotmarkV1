/*****************************************************************
*
* This file is part of the Botmark benchmark.
*
* Copyright (c) 2019 Andrew Murtagh, Patrick Lynch,
* and Conor McGinn.
*
* This work is licensed under the "Creative Commons
* (Attribution-NonCommercial-ShareAlike 4.0 International)
* License" and is copyrighted by Andrew Murtagh, Patrich Lynch,
* and Conor McGinn.
*
* To view a copy of this license, visit
* http://creativecommons.org/licenses/by-nc-sa/4.0/ or
* send a letter to Creative Commons, PO Box 1866,
* Mountain View, CA 94042, USA.
*
* Botmark is distributed in the hope that it will be useful,
* but WITHOUT ANY WARRANTY; without even the implied
* warranty of MERCHANTABILITY or FITNESS FOR A PARTICULAR
* PURPOSE.
*
*****************************************************************/

#include <iostream>
#include <openrave-core.h>
#include <openrave/planningutils.h>
#include <string>

#include <cmath>
#include <boost/thread/thread.hpp>
#include <boost/bind.hpp>

#include <signal.h>

#include <openrave-core.h>
#include <vector>
#include <sstream>
#include <boost/format.hpp>

#include "botmarkcommon.h"

using namespace OpenRAVE;


struct timespec start_time;
struct timespec end_time;




void viewerThread(EnvironmentBasePtr penv, const std::string& viewername) {

	ViewerBasePtr viewer = RaveCreateViewer(penv,viewername);
	//BOOST_ASSERT(!!viewer);

	std::vector<RaveVector<float> > vpoints;
	RaveTransform<float> camera_transform(RaveVector<float>(0.4938367f, -0.6908089f, -0.4205209f, 0.3194848f),
	RaveVector<float>(1.634671f, -0.738299f, 0.907207f));
	viewer->SetCamera(camera_transform, 0.664820f);
	penv->Add(viewer);
	viewer->main(true);

}







int main() {


	std::cout << "Botmark version: " << Botmark_VERSION_MAJOR << "." <<
	Botmark_VERSION_MINOR << "." << Botmark_VERSION_PATCH << std::endl;
	if(BENCHMARK_RUNS < 4) {
        std::cout << "[Botmark Error] Number of runs is less than 4, please change in botmarkcommon.h" << std::endl << std::endl;
		return -1;
	}



	std::cout << "Starting Motion Planning Workload." << std::endl << std::endl;

	std::ofstream results_file;
	results_file.open(RESULTS_FILE.c_str(), std::ofstream::app);
	if(!results_file) {
		std::cout << "[Botmark Error] Failed to open results file." << std::endl;
		return -1;
	}
	double times[BENCHMARK_RUNS];


	//init openrave
	RaveInitialize(true, Level_Fatal);
	EnvironmentBasePtr penv = RaveCreateEnvironment();
	PlannerBasePtr planner = RaveCreatePlanner(penv,"birrt");

	penv->Load(MOTION_PLANNING_ENV_PATH);

	if(VISUALISE_MODE)
		boost::thread viewerthread(boost::bind(viewerThread, penv, "qtcoin"));


	std::vector<RobotBasePtr> vrobots;
	penv->GetRobots(vrobots);
	RobotBasePtr probot = vrobots.at(0);
	RobotBase::ManipulatorPtr pmanip = probot->GetManipulators().at(0);
	for(size_t i = 1; i < probot->GetManipulators().size(); ++i) {
		if(pmanip->GetArmIndices().size() < probot->GetManipulators()[i]->GetArmIndices().size()) {
			pmanip = probot->GetManipulators()[i];
		}
	}






	//start benchmarking runs
	for(int run=0; run<BENCHMARK_RUNS; run++) {

		if(VERBOSE_MODE)
			std::cout << "Run: " << run+1 << " of " << BENCHMARK_RUNS << std::endl;



		//configure manipulator
		std::vector<dReal> initialState;
		initialState.resize(probot->GetActiveDOF());
		initialState[0] = -1.57;
		initialState[1] = 1.35;
		initialState[2] = 0;
		initialState[3] = 1.57;
		initialState[4] = 0;
		initialState[5] = -1.35;
		initialState[6] = 1.57;

		initialState[7] = 1.57;
		initialState[8] = 1.57;
		initialState[9] = 1.57;
		initialState[10] = 0;
		probot->SetDOFValues(initialState);

		PlannerBase::PlannerParametersPtr params(new PlannerBase::PlannerParameters());
		params->_nMaxIterations = 4000;
		params->SetRobotActiveJoints(probot);
		params->vgoalconfig.resize(probot->GetActiveDOF());
		params->vinitialconfig.resize(probot->GetActiveDOF());


		std::vector<dReal> vlower,vupper;
		probot->GetActiveDOFLimits(vlower, vupper);
		RobotBase::RobotStateSaver saver(probot);
		params->vinitialconfig = initialState;
		params->vgoalconfig[0] = 1.57;
		params->vgoalconfig[1] = 0.85;
		params->vgoalconfig[2] = 0;
		params->vgoalconfig[3] = 1.9;
		params->vgoalconfig[4] = 0;
		params->vgoalconfig[5] = -1.1;
		params->vgoalconfig[6] = 1.57;
		params->vgoalconfig[7] = 1.57;
		params->vgoalconfig[8] = 1.57;
		params->vgoalconfig[9] = 1.57;
		params->vgoalconfig[10] = 0;

		probot->GetActiveDOFValues(params->vinitialconfig);



		if(!planner->InitPlan(probot,params)) {
			std::cout << "[Botmark Error] Failed to initialise planner." << std::endl;
			return -1;
		}

		// create a new output trajectory
		TrajectoryBasePtr ptraj = RaveCreateTrajectory(penv,"");

		if(VERBOSE_MODE)
			std::cout << "Starting to plan." << std::endl;


		//start clock
		if(clock_gettime(CLOCK_MONOTONIC_RAW, &start_time)) {
			std::cout << "[Botmark Error] Can't use clock timing routine." << std::endl;
			return -1;
		}

		if(!planner->PlanPath(ptraj)) {
			std::cout << "[Botmark Error] Failed to plan a path." << std::endl;
			return -1;
		}


		//stop clock
		if(clock_gettime(CLOCK_MONOTONIC_RAW, &end_time)) {
			std::cout << "[Botmark Error] Can't use clock timing routine." << std::endl;
			return -1;
		}


		if(VERBOSE_MODE)
			std::cout << "Finished planning." << std::endl;



		//output time
		double start_milli = (double) 1.0e3*start_time.tv_sec + 1.0e-6*start_time.tv_nsec;
		double end_milli = (double) 1.0e3*end_time.tv_sec + 1.0e-6*end_time.tv_nsec;
		times[run] = end_milli - start_milli;

		if(VERBOSE_MODE)
			std::cout << "Elapsed time [ms]: " << end_milli - start_milli << std::endl << std::endl;



		if(VISUALISE_MODE) {

			//draw the end effector of the trajectory
			GraphHandlePtr pgraph;
			std::vector<RaveVector<float> > vpoints;
			std::vector<dReal> vtrajdata;

			for(dReal ftime = 0; ftime <= ptraj->GetDuration(); ftime += 0.01) {
				ptraj->Sample(vtrajdata,ftime,probot->GetActiveConfigurationSpecification());
				probot->SetActiveDOFValues(vtrajdata);
				vpoints.push_back(pmanip->GetEndEffectorTransform().trans);
			}
			pgraph = penv->drawlinestrip(&vpoints[0].x, vpoints.size(), sizeof(vpoints[0]), 4.0f, RaveVector<float>(0.0f, 0.0f, 255.0f, 1.0f));

			probot->GetController()->SetPath(ptraj);

			//wait for the robot to finish
			while(!probot->GetController()->IsDone()) {
				boost::this_thread::sleep(boost::posix_time::milliseconds(1));
			}

		}




	} //end benchmarking runs




	//compute and output statistics
	run_stats_s stats = computeStats(times, BENCHMARK_RUNS);
	if(VERBOSE_MODE) {
		std::cout << "Statistics" << std::endl;
		std::cout << "Total time: " << stats.total << " [ms]" << std::endl;
		std::cout << "Min. time: " << stats.min << " [ms]" << std::endl;
		std::cout << "Max. time: " << stats.max << " [ms]" << std::endl;
		std::cout << "Range: " << stats.range << " [ms]" << std::endl;
		std::cout << "Mean time: " << stats.mean << " [ms]" << std::endl;
		std::cout << "1st quartile: " << stats.first_quartile << " [ms]" << std::endl;
		std::cout << "Median: " << stats.median << " [ms]" << std::endl;
		std::cout << "3rd quartile: " << stats.third_quartile << " [ms]" << std::endl;
		std::cout << "IQR: " << stats.iqr << " [ms]" << std::endl;
		std::cout << "Pop. Standard Deviation: " << stats.std_dev << " [ms]" << std::endl;
		std::cout << "Writing Results to File." << std::endl << std::endl;
	}



	//write results to file
	results_file << "#Results from Motion Planning workload\n";
	results_file << "Metric: time to produce a motion plan\n";
	results_file << "Runs: " << BENCHMARK_RUNS << "\n";
	results_file << "Unit: [ms]\n\n";


	results_file << "##Raw times:\n";
	for(int j=0; j<BENCHMARK_RUNS; j++) {
		results_file << "run: " << j+1 << ",\t\t\ttime: " << times[j] << " [ms]\n";
	}


	results_file << "\n##Statistics:\n";
	results_file << "Total time: \t\t\t\t" << stats.total << " [ms]\n";
	results_file << "Min. time: \t\t\t\t\t" << stats.min << " [ms]\n";
	results_file << "Max. time: \t\t\t\t\t" << stats.max << " [ms]\n";
	results_file << "Range: \t\t\t\t\t\t" << stats.range << " [ms]\n";
	results_file << "Mean time: \t\t\t\t\t" << stats.mean << " [ms]\n";
	results_file << "1st quartile: \t\t\t\t" << stats.first_quartile << " [ms]\n";
	results_file << "Median: \t\t\t\t\t" << stats.median << " [ms]\n";
	results_file << "3rd quartile: \t\t\t\t" << stats.third_quartile << " [ms]\n";
	results_file << "IQR: \t\t\t\t\t\t" << stats.iqr << " [ms]\n";
	results_file << "Pop. Standard Deviation: \t" << stats.std_dev << " [ms]\n";

	results_file << "\n\n\n";
	results_file.close();




	std::cout << "Ending Motion Planning Workload." << std::endl << std::endl;



	return 0;

}
