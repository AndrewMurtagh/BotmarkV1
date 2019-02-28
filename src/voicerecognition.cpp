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
#include <time.h>
#include <boost/filesystem.hpp>

#include <pocketsphinx.h>
#include <alsa/asoundlib.h>
#include <sndfile.h>

#include "botmarkcommon.h"

#define MODELDIR "/usr/local/share/pocketsphinx/model"

typedef std::vector<boost::filesystem::path> path_vector;

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



	std::cout << "Starting Voice Recognition Planning Workload." << std::endl << std::endl;

	std::ofstream results_file;
	results_file.open(RESULTS_FILE.c_str(), std::ofstream::app);
	if(!results_file) {
		std::cout << "[Botmark Error] Failed to open results file." << std::endl;
		return -1;
	}
	double times[BENCHMARK_RUNS];


    //configure pocketsphinx
    ps_decoder_t *ps;
    cmd_ln_t *config;
    FILE *fh;
    char const *hyp;
    int16 buf[512];
    int32 score;

    config = cmd_ln_init(NULL, ps_args(), TRUE,
        "-hmm", MODELDIR "/en-us/en-us",
        "-lm", MODELDIR "/en-us/en-us.lm.bin",
        "-dict", MODELDIR "/en-us/cmudict-en-us.dict",
        "-logfn", "/dev/null",
        NULL);

    if (config == NULL) {
        std::cout << "[Botmark Error] Failed to create config object." << std::endl;
    	return -1;
    }

    ps = ps_init(config);
    if (ps == NULL) {
        std::cout << "[Botmark Error] Failed to create recognizer." << std::endl;
	   return -1;
    }


	//configure alsa
	static char *device = "default";
	int err;
	snd_pcm_t *handle;
	if ((err=snd_pcm_open(&handle, device, SND_PCM_STREAM_PLAYBACK, 0)) < 0){
		std::cout << "[Botmark Error] Playback open error." << std::endl;
		return -1;
	}
	if ((err =
		snd_pcm_set_params(handle,SND_PCM_FORMAT_S16_LE,
		SND_PCM_ACCESS_RW_INTERLEAVED,1,16000, 1, 10000) ) < 0 ){
		std::cout << "[Botmark Error] Playback open error." << std::endl;
		return -1;
	}



    //initiliase directory paths
    path_vector file_path_vec;
    copy(boost::filesystem::directory_iterator(VOICERECOGNITION_PATH),
        boost::filesystem::directory_iterator(),
        back_inserter(file_path_vec));




    //start benchmarking runs
    for(int run=0; run<BENCHMARK_RUNS; run++) {

        if(VERBOSE_MODE)
            std::cout << "Run: " << run+1 << " of " << BENCHMARK_RUNS << std::endl;



        //cycle through files
        for(path_vector::const_iterator it=file_path_vec.begin();
            it!=file_path_vec.end(); ++it) {

            //open file
            if(VERBOSE_MODE)
                std::cout << "Processing file: " << (*it).string() << std::endl;

            fh = fopen((*it).string().c_str(), "rb");
            if (fh == NULL) {
                std::cout << "[Botmark Error] Unable to open input file." << std::endl;
                return -1;
            }

            //start clock
            if(clock_gettime(CLOCK_MONOTONIC_RAW, &start_time)) {
                std::cout << "[Botmark Error] Can't use clock timing routine." << std::endl;
                return -1;
            }


            //decode sound file
            ps_start_utt(ps);

            while (!feof(fh)) {
                size_t nsamp;
                nsamp = fread(buf, 2, 512, fh);
                ps_process_raw(ps, buf, nsamp, FALSE, FALSE);
            }

            ps_end_utt(ps);
            hyp = ps_get_hyp(ps, &score);


            //stop clock
            if(clock_gettime(CLOCK_MONOTONIC_RAW, &end_time)) {
                std::cout << "[Botmark Error] Can't use clock timing routine." << std::endl;
                return -1;
            }




            if(VISUALISE_MODE) {

							fseek(fh, 0, SEEK_END);
							int sizeInBytes = ftell(fh);
							int numSamples = (int) sizeInBytes / sizeof(short);
							short* samples = new short[numSamples];
							fseek(fh, 0, SEEK_SET);


							int numRead = fread(samples, sizeof(short), numSamples, fh);


							snd_pcm_sframes_t frames;
							frames = snd_pcm_writei(handle, samples, numSamples);
							if (frames < 0)
								frames = snd_pcm_recover(handle, frames, 0);
							if (frames < 0) {
								std::cout << "[Botmark Error] snd_pcm_writei failed." << std::endl;
							}

							delete[] samples;

            }

						//output time
            double start_milli = (double) 1.0e3*start_time.tv_sec + 1.0e-6*start_time.tv_nsec;
            double end_milli = (double) 1.0e3*end_time.tv_sec + 1.0e-6*end_time.tv_nsec;
            time_accumulator += end_milli - start_milli;

            if(VERBOSE_MODE) {
                std::cout << "Recognized: " << hyp << std::endl;
                std::cout << "Elapsed time [ms]: " << end_milli - start_milli << std::endl << std::endl;
            }



            fclose(fh);



        } //end cycling through files

        if(VERBOSE_MODE) {
            std::cout << "Total time: " << time_accumulator << std::endl << std::endl;
        }
        times[run] = time_accumulator;
        time_accumulator = 0;


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
		std::cout << "Pop. Standard Deviation: " << stats.std_dev << " [ms]" << std::endl << std::endl;
	}



	//write results to file
	if(VERBOSE_MODE)
		std::cout << "Writing Results to File." << std::endl << std::endl;

	results_file << "#Results from Voice Recognition workload\n";
	results_file << "Metric: total time to decode 15 audio files\n";
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





	//free up memory
	ps_free(ps);
	cmd_ln_free_r(config);
	snd_pcm_close(handle);



	std::cout << "Ending Voice Recognition Workload" << std::endl << std::endl;





    return 0;
}
