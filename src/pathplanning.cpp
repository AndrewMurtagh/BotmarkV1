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


#include <mrpt/maps/COccupancyGridMap2D.h>
#include <mrpt/nav/planners/PlannerSimple2D.h>
#include <mrpt/system/filesystem.h>
#include <mrpt/poses/CPose2D.h>
#include <mrpt/gui/CDisplayWindow.h>

#include <iostream>


#include "botmarkcommon.h"



struct timespec start_time;
struct timespec end_time;






int main() {


	std::cout << "Botmark version: " << Botmark_VERSION_MAJOR << "." <<
	Botmark_VERSION_MINOR << "." << Botmark_VERSION_PATCH << std::endl;
	if(BENCHMARK_RUNS < 4) {
		std::cout << "[Botmark Error] Number of runs is less than 4, please change in botmarkcommon.h" << std::endl << std::endl;
		return -1;
	}

	std::cout << "Starting Path Planning Workload" << std::endl << std::endl;

	std::ofstream results_file;
	results_file.open(RESULTS_FILE.c_str(), std::ofstream::app);
	if(!results_file) {
		std::cout << "[Botmark Error] Failed to open results file." << std::endl;
		return -1;
	}
	double times[BENCHMARK_RUNS];





	//configure
	mrpt::maps::COccupancyGridMap2D gridmap;
	if(!gridmap.loadFromBitmapFile(PATH_PLANNING_MAP, 0.025)) {
		std::cout << "[Botmark Error] Failed to open map." << std::endl;
		return -1;
	}

	mrpt::nav::PlannerSimple2D pathPlanning;
	pathPlanning.robotRadius = 0.2f;

	std::deque<mrpt::math::TPoint2D> thePath;
	bool notFound;

	mrpt::poses::CPose2D origin(5.2, 2.5, 0);
	mrpt::poses::CPose2D target(-1.5, -4.0, 0);



	//start benchmarking runs
	for(int run=0; run<BENCHMARK_RUNS; run++) {

		if(VERBOSE_MODE)
			std::cout << "Run: " << run+1 << " of " << BENCHMARK_RUNS << std::endl;


		if(VERBOSE_MODE)
			std::cout << "Starting to plan." << std::endl;

		//start clock
		if(clock_gettime(CLOCK_MONOTONIC_RAW, &start_time)) {
			std::cout << "[Botmark Error] Can't use clock timing routine." << std::endl;
			return -1;
		}


		//find path
		pathPlanning.computePath(gridmap, origin, target, thePath, notFound, 100.0f);



		//stop clock
		if(clock_gettime(CLOCK_MONOTONIC_RAW, &end_time)) {
			std::cout << "[Botmark Error] Can't use clock timing routine." << std::endl;
			return -1;
		}

		if(notFound) {
			std::cout << "[Botmark Error] Can't plan path." << std::endl;
			return -1;
		}


		if(VERBOSE_MODE)
			std::cout << "Finished planning." << std::endl;




		//output time
		double start_milli = (double) 1.0e3*start_time.tv_sec + 1.0e-6*start_time.tv_nsec;
		double end_milli = (double) 1.0e3*end_time.tv_sec + 1.0e-6*end_time.tv_nsec;
		times[run] = end_milli - start_milli;

		if(VERBOSE_MODE)
			std::cout << "Elapsed ms: " << end_milli - start_milli << std::endl << std::endl;




	} //end benchmarking runs





	if(VISUALISE_MODE) {
		mrpt::utils::CImage img;
		gridmap.getAsImage(img, false, true);

		int radius = round(pathPlanning.robotRadius / gridmap.getResolution());

		for (std::deque<mrpt::math::TPoint2D>::const_iterator it = thePath.begin(); it != thePath.end(); ++it) {
			img.drawCircle(gridmap.x2idx(it->x), gridmap.getSizeY() - 1 - gridmap.y2idx(it->y),
			radius, mrpt::utils::TColor(0, 0, 255));
		}

		img.cross(gridmap.x2idx(origin.x()), gridmap.getSizeY() - 1 - gridmap.y2idx(origin.y()),
		mrpt::utils::TColor(0x20, 0x20, 0x20), '+', 10);

		img.cross(gridmap.x2idx(target.x()),gridmap.getSizeY() - 1 - gridmap.y2idx(target.y()),
		mrpt::utils::TColor(0x50, 0x50, 0x50), 'x', 10);


		mrpt::gui::CDisplayWindow display("Path planning");
		display.showImage(img);
		std::cout << "Press any key to continue" << std::endl << std::endl;
		display.waitForKey();

	}



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
	results_file << "#Results from Path Planning workload\n";
	results_file << "Metric: time to find a path\n";
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






	std::cout << "Ending Path Planning Workload" << std::endl << std::endl;


	return 0;
}
