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

#ifndef BOTMARK_COMMON_H
#define BOTMARK_COMMON_H

#include <math.h>
#include <algorithm>
#include "BotmarkConfig.h"

#define STRINGIZE2(x) #x
#define STRINGIZE(x) STRINGIZE2(x)


//benchmarking options
const bool VERBOSE_MODE = false;
const bool VISUALISE_MODE = false;
const int BENCHMARK_RUNS = 10;
const std::string RESULTS_FILE = STRINGIZE(ROOT_DIRECTORY)+std::string("/res/results.txt");

//person tracking
const std::string RGB_PATH = STRINGIZE(ROOT_DIRECTORY)+std::string("/data/persontracking/rgb/");
const std::string DEPTH_PATH = STRINGIZE(ROOT_DIRECTORY)+std::string("/data/persontracking/depth/");
const std::string FRONTALFACE_CASCADE_PATH = STRINGIZE(ROOT_DIRECTORY)+std::string("/res/haarcascade_frontalface_alt2.xml");
const std::string HOG_SVM = STRINGIZE(ROOT_DIRECTORY)+std::string("/res/trainedLinearSVMForPeopleDetectionWithHOG.yaml");


//text to speech
const std::string TEXTTOSPEECH_FILE_PATH = STRINGIZE(ROOT_DIRECTORY)+std::string("/res/speech_commands.txt");


//voice recognition
const std::string VOICERECOGNITION_PATH = STRINGIZE(ROOT_DIRECTORY)+std::string("/data/voicerecognition/");
const int NUM_STATEMENTS=15;


//object recognition
const int OBJECT_RECOGNITION_FRAMES = 50;
const std::string CAFFE_MODEL_FILE = STRINGIZE(ROOT_DIRECTORY)+std::string("/res/deploy.prototxt");
const std::string CAFFE_TRAINED_FILE = STRINGIZE(ROOT_DIRECTORY)+std::string("/res/bvlc_reference_caffenet.caffemodel");
const std::string CAFFE_MEAN_FILE = STRINGIZE(ROOT_DIRECTORY)+std::string("/res/imagenet_mean.binaryproto");
const std::string CAFFE_LABEL_FILE = STRINGIZE(ROOT_DIRECTORY)+std::string("/res/synset_words.txt");
const std::string OBJECT_RECOGNITION_DATA_PATH = STRINGIZE(ROOT_DIRECTORY)+std::string("/data/objectrecognition/");

//slam
const std::string SLAM_LOG_PATH = STRINGIZE(ROOT_DIRECTORY)+std::string("/data/slam/intel.log");


//path planning
const std::string PATH_PLANNING_MAP = STRINGIZE(ROOT_DIRECTORY)+std::string("/data/pathplanning/path_planning_map.png");
const std::string PATH_PLANNING_OUTPUT = STRINGIZE(ROOT_DIRECTORY)+std::string("/data/pathplanning/path_planning_output.png");


//motion planning
const std::string MOTION_PLANNING_ENV_PATH = STRINGIZE(ROOT_DIRECTORY)+std::string("/res/botmark.env.xml");


//facial recognition
const std::string FACIAL_RECOGNITION_PATH = STRINGIZE(ROOT_DIRECTORY)+std::string("/data/facialrecognition/");
const std::string FACIAL_RECOGNITION_TRAINING_LABELS = STRINGIZE(ROOT_DIRECTORY)+std::string("/res/facialrecognition_training_labels.csv");
const std::string FACIAL_RECOGNITION_TEST_LABELS = STRINGIZE(ROOT_DIRECTORY)+std::string("/res/facialrecognition_test_labels.csv");
const std::string FACIAL_RECOGNITION_MODEL = STRINGIZE(ROOT_DIRECTORY)+std::string("/res/facialrecognition_model.yaml");


struct run_stats_s {
    double min, max, mean, std_dev, median, total, range, first_quartile, third_quartile, iqr; //sample std dev,
    run_stats_s() : min(1e9), max(0), mean(0), std_dev(0), median(0),
        total(0), range(0), first_quartile(0), third_quartile(0), iqr(0) {};
};




run_stats_s computeStats(double frames[], int size) {
    run_stats_s stats;


    for(int j=0; j<size; j++) {
        stats.total += frames[j];
        if(frames[j] > stats.max)
            stats.max = frames[j];
        if(frames[j] < stats.min)
            stats.min = frames[j];
    }
    stats.mean = stats.total/(double)size;
    stats.range = stats.max - stats.min;

    for(int j=0; j<size; j++) {
        stats.std_dev += pow(frames[j] - stats.mean, 2);
    }

    stats.std_dev = sqrt(stats.std_dev/(double)size);


    //median
    std::sort(frames, frames + size);


    if(size%2 == 0) {
        stats.first_quartile = double(frames[size/4]);
    } else {
        stats.first_quartile = double(frames[size/4-1] + frames[(size/4)])*0.5;
    }

 	if(size%2 == 0) {
        stats.median = double(frames[size/2] + frames[(size/2)-1])*0.5;
    } else {
        stats.median = double(frames[size/2]);
    }




    if(size%2== 0) {
        stats.third_quartile = double(frames[size*3/4]);
    } else {
        stats.third_quartile = double(frames[size*3/4] + frames[(size*3/4+1)])*0.5;
    }

    stats.iqr = stats.third_quartile - stats.first_quartile;


    return stats;
}




#endif //BOTMARK_COMMON_H
