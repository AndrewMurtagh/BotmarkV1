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
#include <sstream>

#include "opencv2/core.hpp"
#include "opencv2/face.hpp"
#include "opencv2/highgui.hpp"
#include "opencv2/imgproc.hpp"


#include "botmarkcommon.h"

using namespace cv;
using namespace cv::face;
using namespace std;



struct timespec start_time;
struct timespec end_time;
float time_accumulator=0.0f;



int main(int argc, const char *argv[]) {


 	std::cout << "Botmark version: " << Botmark_VERSION_MAJOR << "." <<
 	Botmark_VERSION_MINOR << "." << Botmark_VERSION_PATCH << std::endl;
 	if(BENCHMARK_RUNS < 4) {
 		std::cout << "[Botmark Error] Number of runs is less than 4, please change in botmarkcommon.h" << std::endl << std::endl;
 		return -1;
 	}

 	std::cout << "Starting Facial Recognition Workload" << std::endl << std::endl;

 	std::ofstream results_file;
 	results_file.open(RESULTS_FILE.c_str(), std::ofstream::app);
 	if(!results_file) {
 		std::cout << "[Botmark Error] Failed to open results file." << std::endl;
 		return -1;
 	}
 	double times[BENCHMARK_RUNS];




    //configure
    int NUM_FRAMES=723;

    std::ifstream file(FACIAL_RECOGNITION_TEST_LABELS.c_str(), ifstream::in);
    if (!file) {
        std::cout << "[Botmark Error] could not open file." << std::endl;
        return -1;
    }

    Ptr<FaceRecognizer> model = FisherFaceRecognizer::create();
    model->read(FACIAL_RECOGNITION_MODEL);



    //start benchmarking runs
    for(int run=0; run<BENCHMARK_RUNS; run++) {

        if(VERBOSE_MODE)
        std::cout << "Run: " << run+1 << " of " << BENCHMARK_RUNS << std::endl;



        //cycle through frames
        for(size_t i=0; i<NUM_FRAMES; i++) {


            if(VERBOSE_MODE)
                std::cout << "Processing frame " << i+1 << " of " << NUM_FRAMES << std::endl;


            //read in image
            Mat this_image;
            int this_label;

            string line, path, classlabel;
            getline(file, line);
            stringstream liness(line);
            getline(liness, path, ';');
            getline(liness, classlabel);
            path = FACIAL_RECOGNITION_PATH+"testing/"+path;
            if(!path.empty() && !classlabel.empty()) {
                this_image = imread(path, CV_LOAD_IMAGE_GRAYSCALE);
                this_label = atoi(classlabel.c_str());
                //images.push_back(imread(path, CV_LOAD_IMAGE_GRAYSCALE));
                //labels.push_back(atoi(classlabel.c_str()));
            }




            //start clock
            if(clock_gettime(CLOCK_MONOTONIC_RAW, &start_time)) {
                std::cout << "[Botmark Error] Can't use clock timing routine" << std::endl;
                return -1;
            }

            int predictedLabel = model->predict(this_image);


            //stop clock
            if(clock_gettime(CLOCK_MONOTONIC, &end_time)) {
                std::cout << "[Botmark Error] Can't use clock timing routine" << std::endl;
                return -1;
            }

            //calculate time
            double start_milli = (double) 1.0e3*start_time.tv_sec + 1.0e-6*start_time.tv_nsec;
            double end_milli = (double) 1.0e3*end_time.tv_sec + 1.0e-6*end_time.tv_nsec;
            time_accumulator += end_milli - start_milli;



            string result_message = format("Predicted: %d , Actual: %d", predictedLabel, this_label);
            if(VERBOSE_MODE) {
                std::cout << result_message << std::endl;
                std::cout << "Elapsed time [ms]: " << end_milli - start_milli << std::endl << std::endl;
            }





            if(VISUALISE_MODE) {
                putText(this_image, result_message, Point(10,20), FONT_HERSHEY_PLAIN, 1, cv::Scalar(255, 0, 0));
                namedWindow("FacialRecognition", WINDOW_AUTOSIZE);
                imshow("FacialRecognition", this_image);
                waitKey(25);

            }





        } //end cycling through frames


        float avr_time_per_frame = time_accumulator/NUM_FRAMES;
        if(VERBOSE_MODE) {
            std::cout << "Frame rate [Hz]: " << 1000.0f/avr_time_per_frame << std::endl << std::endl;
        }
        times[run] = 1000.0f/avr_time_per_frame;


        time_accumulator = 0;
        file.clear();
        file.seekg(0, ios::beg);


    } //end benchmarking runs



    //compute and output statistics
    run_stats_s stats = computeStats(times, BENCHMARK_RUNS);
    if(VERBOSE_MODE) {
        std::cout << "Statistics" << std::endl;
        std::cout << "Min. frame rate: " << stats.min << " [ms]" << std::endl;
        std::cout << "Max. frame rate: " << stats.max << " [ms]" << std::endl;
        std::cout << "Range: " << stats.range << " [ms]" << std::endl;
        std::cout << "Mean frame rate: " << stats.mean << " [ms]" << std::endl;
        std::cout << "1st quartile: " << stats.first_quartile << " [ms]" << std::endl;
        std::cout << "Median: " << stats.median << " [ms]" << std::endl;
        std::cout << "3rd quartile: " << stats.third_quartile << " [ms]" << std::endl;
        std::cout << "IQR: " << stats.iqr << " [ms]" << std::endl;
        std::cout << "Pop. Standard Deviation: " << stats.std_dev << " [ms]" << std::endl;
        std::cout << "Writing Results to File." << std::endl << std::endl;
    }

    //write results to file
    results_file << "#Results from Facial Recognition workload\n";
    results_file << "Metric: time to recognise a person in a frame\n";
    results_file << "Runs: " << BENCHMARK_RUNS << "\n";
    results_file << "Unit: [Hz]\n\n";


    results_file << "##Raw frame rates:\n";
    for(int j=0; j<BENCHMARK_RUNS; j++) {
        results_file << "run: " << j+1 << ",\t\t\tframe rate: " << times[j] << " [Hz]\n";
    }


    results_file << "\n##Statistics:\n";
    results_file << "Min. frame rate: \t\t\t" << stats.min << " [Hz]\n";
    results_file << "Max. frame rate: \t\t\t" << stats.max << " [Hz]\n";
    results_file << "Range: \t\t\t\t\t\t" << stats.range << " [Hz]\n";
    results_file << "Mean frame rate: \t\t\t" << stats.mean << " [Hz]\n";
    results_file << "1st quartile: \t\t\t\t" << stats.first_quartile << " [Hz]\n";
    results_file << "Median: \t\t\t\t\t" << stats.median << " [Hz]\n";
    results_file << "3rd quartile: \t\t\t\t" << stats.third_quartile << " [Hz]\n";
    results_file << "IQR: \t\t\t\t\t\t" << stats.iqr << " [Hz]\n";
    results_file << "Pop. Standard Deviation: \t" << stats.std_dev << " [Hz]\n";

    results_file << "\n\n\n";
    results_file.close();


    std::cout << "Ending Facial Recognition Workload" << std::endl << std::endl;





    return 0;
}
