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
#include <fstream>
#include <iostream>
#include <sstream>


#include "botmarkcommon.h"

#include <log/carmenconfiguration.h>
#include <log/sensorstream.h>
#include <gridfastslam/gridslamprocessor.h>
#include <utils/commandline.h>
#include <utils/stat.h>
#include <configfile/configfile.h>

#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>

using namespace GMapping;
using namespace std;


struct timespec start_time;
struct timespec end_time;

class NullBuffer : public std::streambuf
{
public:
	int overflow(int c) { return c; }
};


int main(int argc, const char *argv[]) {

	std::cout << "Botmark version: " << Botmark_VERSION_MAJOR << "." <<
 	Botmark_VERSION_MINOR << "." << Botmark_VERSION_PATCH << std::endl;
 	if(BENCHMARK_RUNS < 4) {
 		std::cout << "[Botmark Error] Number of runs is less than 4, please change in botmarkcommon.h" << std::endl << std::endl;
 		return -1;
 	}

 	std::cout << "Starting SLAM Workload" << std::endl << std::endl;

 	std::ofstream results_file;
 	results_file.open(RESULTS_FILE.c_str(), std::ofstream::app);
 	if(!results_file) {
 		std::cout << "[Botmark Error] Failed to open results file." << std::endl;
 		return -1;
 	}
 	double times[BENCHMARK_RUNS];
	long reading_count=0;
	double time_accumulator=0.0;



	NullBuffer null_buffer;
	std::ostream null_stream(&null_buffer);


    //configure
	double xmin=-100.;
	double ymin=-100.;
	double xmax=100.;
	double resampleThreshold=0.5;
	double ymax=100.;
	double delta=0.05;
	double sigma=0.05;
	double maxrange=80.;
	double maxUrange=80.;
	double regscore=1e4;
	double lstep=.05;
	double astep=.05;
	int kernelSize=1;
	int iterations=5;
	double lsigma=.075;
	double ogain=3;
	unsigned int lskip=0;
	double llsamplerange=0.01;
	double llsamplestep=0.01;
	double lasamplerange=0.005;
	double lasamplestep=0.005;
	int particles=30;
	double angularUpdate=0.5;
	double linearUpdate=1;
	double srr=0.1;
	double srt=0.1;
	double str=0.1;
	double stt=0.1;
	bool generateMap=true;
	unsigned int randseed=0;


	SensorMap sensorMap;
	GridSlamProcessor gsp(null_stream);
	InputSensorStream* input;
	OrientedPoint initialPose(0,0,0);


	ifstream config_file;
	ifstream log_stream;
	config_file.open(SLAM_LOG_PATH.c_str());
	if(!config_file) {
        std::cout << "[Botmark Error] could not open file." << std::endl;
        return -1;
    }
	CarmenConfiguration conf;
	conf.load(config_file);
	config_file.close();
	log_stream.open(SLAM_LOG_PATH.c_str());

	sensorMap=conf.computeSensorMap();


	input=new InputSensorStream(sensorMap, log_stream);


	gsp.setSensorMap(sensorMap);
	gsp.setMatchingParameters(maxUrange, maxrange, sigma, kernelSize, lstep, astep, iterations, lsigma, ogain, lskip);
	gsp.setMotionModelParameters(srr, srt, str, stt);
	gsp.setUpdateDistances(linearUpdate, angularUpdate, resampleThreshold);
	gsp.setgenerateMap(generateMap);
	gsp.GridSlamProcessor::init(particles, xmin, ymin, xmax, ymax, delta, initialPose);
	gsp.setllsamplerange(llsamplerange);
	gsp.setllsamplestep(llsamplestep);
	gsp.setlasamplerange(llsamplerange);
	gsp.setlasamplestep(llsamplestep);


  //start benchmarking runs
  for(int run=0; run<BENCHMARK_RUNS; run++) {

    if(VERBOSE_MODE)
        std::cout << "Run: " << run+1 << " of " << BENCHMARK_RUNS << std::endl;





		while(*(input)) {





			const SensorReading* r;

			(*(input)) >> r;
			if(!r) {
				continue;
			} else {
				++reading_count;
			}

			//if(VERBOSE_MODE)
        //std::cout << "Processing sensor reading " << reading_count << std::endl;

			// start clock
      if(clock_gettime(CLOCK_MONOTONIC_RAW, &start_time)) {
          std::cout << "[Botmark Error] Can't use clock timing routine" << std::endl;
          return -1;
      }

			const RangeReading* rr=dynamic_cast<const RangeReading*>(r);
			if(rr){
				const RangeSensor* rs=dynamic_cast<const RangeSensor*>(rr->getSensor());
				gsp.processScan(*rr);
			}

			const OdometryReading* o=dynamic_cast<const OdometryReading*>(r);
			if(o){
				gsp.processTruePos(*o);
			}

			//stop clock
	    if(clock_gettime(CLOCK_MONOTONIC_RAW, &end_time)) {
        std::cout << "[Botmark Error] Can't use clock timing routine" << std::endl;
        return -1;
	    }

      //calculate time
			double start_milli = (double) 1.0e3*start_time.tv_sec + 1.0e-6*start_time.tv_nsec;
			double end_milli = (double) 1.0e3*end_time.tv_sec + 1.0e-6*end_time.tv_nsec;
			time_accumulator += end_milli - start_milli;


			if(VERBOSE_MODE) {
				std::cout << "Elapsed time [ms]: " << end_milli - start_milli << std::endl << std::endl;
			}

			if(VISUALISE_MODE && reading_count%150==0) { //only display every 150 readings
				ScanMatcherMap smap = ScanMatcherMap(gsp.getParticles()[gsp.getBestParticleIndex()].map);
				cv::Mat map(smap.getMapSizeX(), smap.getMapSizeY(), CV_8UC3);
				for(int x=0; x < smap.getMapSizeX(); x++) {
					for(int y=0; y < smap.getMapSizeY(); y++) {
						double occ=smap.cell(x,y);
						if(occ<0) {
						    map.at<cv::Vec3b>(cv::Point(x,y)) = cv::Vec3f(127, 127, 127);
						} else if(occ>=0.5) {
						    map.at<cv::Vec3b>(cv::Point(x,y)) = cv::Vec3f(0, 0, 0);
						} else {
						    map.at<cv::Vec3b>(cv::Point(x,y)) = cv::Vec3f(255, 255, 255);
						}
					}
				}

				OrientedPoint pose = gsp.getParticles()[gsp.getBestParticleIndex()].pose;
				cv::circle(map, cv::Point(smap.world2map(pose.x, pose.y).x, smap.world2map(pose.x, pose.y).y), 5, cv::Scalar(0,0,255), -1);

				cv::resize(map, map, cv::Size(1600, 1600), 0, 0, cv::INTER_CUBIC);
				map = map(cv::Rect(400, 200, 800, 800));
				cv::namedWindow("temp", cv::WINDOW_AUTOSIZE);
				cv::imshow("temp", map);
				cv::waitKey(1);

			}




		} // end cycling through readings

		//std::cout << "total " << time_accumulator << std::endl;
		//std::cout << "reading_count " << reading_count << std::endl;
		double avr_time_per_sensor = time_accumulator/(double)reading_count;
		if(VERBOSE_MODE) {
			std::cout << "Average time [ms]: " << avr_time_per_sensor << std::endl << std::endl;
		}
		times[run] = avr_time_per_sensor;


		time_accumulator = 0;
		reading_count = 0;
		log_stream.clear();
		log_stream.seekg(0, ios::beg);
		gsp.GridSlamProcessor::init(particles, xmin, ymin, xmax, ymax, delta, initialPose);
		input=new InputSensorStream(sensorMap, log_stream);


  } //end benchmarking runs


	//compute and output statistics
	run_stats_s stats = computeStats(times, BENCHMARK_RUNS);
	if(VERBOSE_MODE) {
			std::cout << "Statistics" << std::endl;
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
	results_file << "#Results from SLAM workload\n";
	results_file << "Metric: Time to process a sensor reading\n";
	results_file << "Runs: " << BENCHMARK_RUNS << "\n";
	results_file << "Unit: [ms]\n\n";


	results_file << "##Raw times:\n";
	for(int j=0; j<BENCHMARK_RUNS; j++) {
			results_file << "run: " << j+1 << ",\t\t\ttime: " << times[j] << " [ms]\n";
	}


	results_file << "\n##Statistics:\n";
	results_file << "Min. time: \t\t\t" << stats.min << " [ms]\n";
	results_file << "Max. time: \t\t\t" << stats.max << " [ms]\n";
	results_file << "Range: \t\t\t\t\t\t" << stats.range << " [ms]\n";
	results_file << "Mean time: \t\t\t" << stats.mean << " [ms]\n";
	results_file << "1st quartile: \t\t\t\t" << stats.first_quartile << " [ms]\n";
	results_file << "Median: \t\t\t\t\t" << stats.median << " [ms]\n";
	results_file << "3rd quartile: \t\t\t\t" << stats.third_quartile << " [ms]\n";
	results_file << "IQR: \t\t\t\t\t\t" << stats.iqr << " [ms]\n";
	results_file << "Pop. Standard Deviation: \t" << stats.std_dev << " [ms]\n";

	results_file << "\n\n\n";
	results_file.close();



	std::cout << "Ending SLAM Workload" << std::endl << std::endl;




	return 0;
}
