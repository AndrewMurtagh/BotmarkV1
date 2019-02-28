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
#include <string>

#include <festival/festival.h>

#include <iostream>
#include <time.h>
#include "botmarkcommon.h"


struct timespec start_time;
struct timespec end_time;
float time_accumulator=0.0f;


int main() {


    std::cout << "Botmark version: " << Botmark_VERSION_MAJOR << "." <<
    Botmark_VERSION_MINOR << "." << Botmark_VERSION_PATCH << std::endl;
    if(BENCHMARK_RUNS < 4) {
        std::cout << "[Botmark Error] Number of runs is less than 4, please change in botmarkcommon.h" << std::endl << std::endl;
        return -1;
    }



    std::cout << "Starting Speech Synthesis Workload." << std::endl << std::endl;

    std::ofstream results_file;
    results_file.open(RESULTS_FILE.c_str(), std::ofstream::app);
    if(!results_file) {
        std::cout << "[Botmark Error] Failed to open results file." << std::endl;
        return -1;
    }
    double times[BENCHMARK_RUNS];


    //configure
    int heap_size = 2100000;
    int load_init_files = 1;
    festival_initialize(load_init_files,heap_size);
    std::ifstream file(TEXTTOSPEECH_FILE_PATH.c_str());
    std::string str_command;



    //start benchmarking runs
    for(int run=0; run<BENCHMARK_RUNS; run++) {

        if(VERBOSE_MODE)
            std::cout << "Run: " << run+1 << " of " << BENCHMARK_RUNS << std::endl;



        //cycle through commands
        while (std::getline(file, str_command)) {


            //output message
            if(VERBOSE_MODE)
                std::cout << "Processing command: " << str_command << std::endl;

            //start clock
            if(clock_gettime(CLOCK_MONOTONIC_RAW, &start_time)) {
                std::cout << "[Botmark Error] Can't use clock timing routine." << std::endl;
                return -1;
            }

            //produce speech
            festival_say_text(str_command.c_str());


            //stop clock
            if(clock_gettime(CLOCK_MONOTONIC_RAW, &end_time)) {
                std::cout << "[Botmark Error] Can't use clock timing routine." << std::endl;
                return -1;
            }

            //output time
            double start_milli = (double) 1.0e3*start_time.tv_sec + 1.0e-6*start_time.tv_nsec;
            double end_milli = (double) 1.0e3*end_time.tv_sec + 1.0e-6*end_time.tv_nsec;
            time_accumulator += end_milli - start_milli;

            if(VERBOSE_MODE) {
                std::cout << "Elapsed time [ms]: " << end_milli - start_milli << std::endl << std::endl;
            }


        } //end cycling through commands

        if(VERBOSE_MODE) {
            std::cout << "Total time: " << time_accumulator << std::endl << std::endl;
        }
        times[run] = time_accumulator;


        time_accumulator = 0;
        file.clear();
        file.seekg(0, ios::beg);


    } //end cycling through benchmark runs



    file.close();



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
    results_file << "#Results from Speech Synthesis workload\n";
    results_file << "Metric: total time to produce 15 speech commands\n";
    results_file << "Runs: " << BENCHMARK_RUNS << "\n";
    results_file << "Unit: [ms]\n\n";


    results_file << "##Raw times:\n";
    for(int j=0; j<BENCHMARK_RUNS; j++) {
        results_file << "run: " << j+1 << ",\t\t\ttotal time: " << times[j] << " [ms]\n";
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







    std::cout << "Ending Speech Synthesis Workload" << std::endl << std::endl;





    return 0;
}
