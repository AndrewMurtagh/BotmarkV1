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
#include <vector>
#include <boost/filesystem.hpp>


#include <opencv2/opencv.hpp>
#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>


#include <pcl/common/transforms.h>
#include <pcl/people/ground_based_people_detection_app.h>
#include <pcl/segmentation/extract_clusters.h>
#include <pcl/filters/extract_indices.h>
#include <pcl/visualization/pcl_visualizer.h>

#include "botmarkcommon.h"

using namespace pcl;
using namespace cv;

typedef std::vector<boost::filesystem::path> path_vector;
typedef PointXYZRGBA PointT;
typedef PointCloud<PointT> PointCloudT;


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

 	std::cout << "Starting Person Tracking Workload" << std::endl << std::endl;

 	std::ofstream results_file;
 	results_file.open(RESULTS_FILE.c_str(), std::ofstream::app);
 	if(!results_file) {
 		std::cout << "[Botmark Error] Failed to open results file." << std::endl;
 		return -1;
 	}
 	double times[BENCHMARK_RUNS];



    //configure
    int NUM_FRAMES=250;


    Mat rgbimage, grayscaleimage;
    boost::filesystem::path rgb_path(RGB_PATH);
    path_vector rgb_path_vec;
    copy(boost::filesystem::directory_iterator(rgb_path), boost::filesystem::directory_iterator(), back_inserter(rgb_path_vec));
    sort(rgb_path_vec.begin(), rgb_path_vec.end());
    path_vector::const_iterator rgb_iterator(rgb_path_vec.begin());

    HOGDescriptor hog;
    hog.setSVMDetector(HOGDescriptor::getDefaultPeopleDetector());
    CascadeClassifier face_cascade;
    if(!face_cascade.load(FRONTALFACE_CASCADE_PATH)) {
        std::cout << "[Botmark Error] Could not load cascades." << std::endl;
        return false;
    };
    cv::Scalar BLUE(255, 0, 0); //B,G,R
    cv::Scalar GREEN(0, 255, 0);
    cv::Scalar RED(0, 0, 255);
    cv::Scalar CYAN(255, 255, 0);


    float KINECT_FOCAL_LENGTH = 575.8157496f;
    int POINT_CLOUD_SCALING= 1000; // m to mm
    float voxel_size = 0.06;
    Eigen::Matrix3f rgb_intrinsics_matrix;
    rgb_intrinsics_matrix << 525.0f, 0.0f, 319.5f, 0.0f, 525.0f, 239.5f, 0.0f, 0.0f, 1.0f; // Kinect RGB camera intrinsics

    Mat depthimage;
    boost::filesystem::path depth_path(DEPTH_PATH);
    path_vector depth_path_vec;
    copy(boost::filesystem::directory_iterator(depth_path), boost::filesystem::directory_iterator(), back_inserter(depth_path_vec));
    sort(depth_path_vec.begin(), depth_path_vec.end());
    path_vector::const_iterator depth_iterator(depth_path_vec.begin());

    PointCloudT::Ptr cloud(new PointCloudT);

    pcl::visualization::PointCloudColorHandlerCustom<PointT> cloud_color_handler(cloud, 255, 255, 255);
    pcl::visualization::PCLVisualizer::Ptr viewer;
    if(VISUALISE_MODE) {
        viewer = boost::make_shared<pcl::visualization::PCLVisualizer>("Viewer");
        viewer->setCameraPosition(0,0,-2,0,-1,0,0);
        viewer->addCoordinateSystem (1.0);
        viewer->setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 1, "cloud");
    }



    PointCloudT::Ptr ground_plane_points (new PointCloudT);
    PointT current_point;
    current_point.x = 1.06109;
    current_point.y = 0.876872;
    current_point.z = 4.243;
    ground_plane_points->points.push_back(current_point);
    current_point.x = 0.943696;
    current_point.y = 0.988163;
    current_point.z = 2.845;
    ground_plane_points->points.push_back(current_point);
    current_point.x = -0.825116;
    current_point.y = 0.963459;
    current_point.z = 2.845;
    ground_plane_points->points.push_back(current_point);
    //viewer.addSphere(current_point, 0.1, 255, 0, 0, "one");
    //viewer.addSphere(current_point, 0.1, 0, 0, 255, "three");
    //viewer.addSphere(current_point, 0.1, 0, 255, 0, "two");

    pcl::people::PersonClassifier<pcl::RGB> person_classifier;
    person_classifier.loadSVMFromFile(HOG_SVM);

    Eigen::VectorXf ground_coeffs;
    ground_coeffs.resize(4);
    std::vector<int> clicked_points_indices;
    for (unsigned int i = 0; i < ground_plane_points->points.size(); i++)
    clicked_points_indices.push_back(i);
    pcl::SampleConsensusModelPlane<PointT> model_plane(ground_plane_points);
    model_plane.computeModelCoefficients(clicked_points_indices, ground_coeffs);


    pcl::people::GroundBasedPeopleDetectionApp<PointT> people_detector;
    people_detector.setVoxelSize(voxel_size);
    people_detector.setIntrinsics(rgb_intrinsics_matrix);
    people_detector.setClassifier(person_classifier);




    //start benchmarking runs
    for(int run=0; run<BENCHMARK_RUNS; run++) {

        if(VERBOSE_MODE)
        std::cout << "Run: " << run+1 << " of " << BENCHMARK_RUNS << std::endl;



        //cycle through frames
        for(int i=0; i<NUM_FRAMES; i++) {


            if(VERBOSE_MODE)
                std::cout << "Processing frame " << i+1 << " of " << NUM_FRAMES << std::endl;

            //start clock
            if(clock_gettime(CLOCK_MONOTONIC_RAW, &start_time)) {
                std::cout << "[Botmark Error] Can't use clock timing routine" << std::endl;
                return -1;
            }



            //rgb processing
            std::vector<Rect> hog_features, haar_face_features;
            rgbimage = imread((*rgb_iterator).string(), IMREAD_COLOR);
            if(rgbimage.empty()) {
                std::cout << "[Botmark Error] Could not load rgb image" << std::endl;
                return false;
            }

            hog.detectMultiScale(rgbimage, hog_features, 0, Size(8,8), Size(16,16), 1.04, 4);
            cvtColor(rgbimage, grayscaleimage, CV_BGR2GRAY);
            equalizeHist(grayscaleimage, grayscaleimage);
            face_cascade.detectMultiScale(rgbimage, haar_face_features, 1.04, 5, 0|CV_HAAR_SCALE_IMAGE, Size(20,20), Size(80,80));




            //depth processing
            Rect pc_feature;
            depthimage = imread((*depth_iterator).string(), IMREAD_ANYDEPTH);
            if(depthimage.empty()) {
                std::cout << "[Botmark Error] Could not load depth image" << std::endl;
                return false;
            }

            //convert depth image to p.c.
            int width = depthimage.cols;
            int height = depthimage.rows;
            cloud->height = height;
            cloud->width = width;
            cloud->points.resize(cloud->height * cloud->width);

            int centerX = (cloud->width >> 1);
            int centerY = (cloud->height >> 1);
            int depth_idx = 0;
            for(int v = -centerY; v < centerY; ++v) {
                for(int u = -centerX; u < centerX; ++u, ++depth_idx) {
                    PointT& pt = cloud->points[depth_idx];
                    pt.z = (float) depthimage.at<unsigned short>(Point(u+centerX, v+centerY));
                    pt.x = static_cast<float>(u)*pt.z*(1.0f/KINECT_FOCAL_LENGTH);
                    pt.y = static_cast<float>(v)*pt.z*(1.0f/KINECT_FOCAL_LENGTH);

                    pt.z/=POINT_CLOUD_SCALING;
                    pt.x/=POINT_CLOUD_SCALING;
                    pt.y/=POINT_CLOUD_SCALING;
                }
            }

            std::vector<pcl::people::PersonCluster<PointT> > clusters;
            people_detector.setInputCloud(cloud);
            people_detector.setGround(ground_coeffs);
            people_detector.compute(clusters);
            ground_coeffs = people_detector.getGround();

            float min_distance = 10000.0f; //large number
            int min_distance_index=0;
            for(std::vector<pcl::people::PersonCluster<PointT> >::iterator it = clusters.begin(); it != clusters.end(); ++it) {
                if(it->getDistance() < min_distance) {
                    min_distance_index = it - clusters.begin();
                }
            }
            PointT min_point;
            min_point.x = clusters[min_distance_index].getMin()[0];
            min_point.y = clusters[min_distance_index].getMin()[1];
            min_point.z = clusters[min_distance_index].getMin()[2];
            PointT max_point;
            max_point.x = clusters[min_distance_index].getMax()[0];
            max_point.y = clusters[min_distance_index].getMax()[1];
            max_point.z = clusters[min_distance_index].getMax()[2];

            float min_raster_u = (min_point.x)*(KINECT_FOCAL_LENGTH/min_point.z) + centerX;
            float min_raster_v = (min_point.y)*(KINECT_FOCAL_LENGTH/min_point.z) + centerY;
            float max_raster_u = (max_point.x)*(KINECT_FOCAL_LENGTH/max_point.z) + centerX;
            float max_raster_v = (max_point.y)*(KINECT_FOCAL_LENGTH/max_point.z) + centerY;
            pc_feature = Rect(Point(min_raster_u,min_raster_v), Point(max_raster_u,max_raster_v));




            //multi-modal fusion using a simple scoring scheme
            //should use somthing like non-maximal suppression

            Mat mask(rgbimage.size(), CV_8UC1, Scalar(0));
            for(int i=0; i<mask.rows; i++) {
                for(int j=0; j<mask.cols; j++) {
                    if(pc_feature.contains(Point(i,j))) {
                        mask.at<uchar>(Point(i,j)) += 1;
                    }
                    for(size_t k = 0; k < haar_face_features.size(); k++) {
                        if(haar_face_features[k].contains(Point(i,j))) {
                            mask.at<uchar>(Point(i,j)) += 1;
                        }
                    }
                    for(size_t k = 0; k < hog_features.size(); k++) {
                        if(hog_features[k].contains(Point(i,j))) {
                            mask.at<uchar>(Point(i,j)) += 1;
                        }
                    }
                }
            }

            normalize(mask, mask, 0, 255, cv::NORM_MINMAX);
            threshold(mask, mask, 254, 255, THRESH_BINARY);

            std::vector<std::vector<cv::Point> > contours;
            cv::findContours(mask,contours,CV_RETR_TREE,CV_CHAIN_APPROX_SIMPLE);
            Rect final_rect;
            for (size_t i=0; i<contours.size(); i++) {
                    final_rect = final_rect | cv::boundingRect(cv::Mat(contours[i]));
            }
            Point person_centroid = (final_rect.br() + final_rect.tl())*0.5;



            //stop clock
            if(clock_gettime(CLOCK_MONOTONIC_RAW, &end_time)) {
                std::cout << "[Botmark Error] Can't use clock timing routine" << std::endl;
                return -1;
            }


            //calculate time
            double start_milli = (double) 1.0e3*start_time.tv_sec + 1.0e-6*start_time.tv_nsec;
            double end_milli = (double) 1.0e3*end_time.tv_sec + 1.0e-6*end_time.tv_nsec;
            time_accumulator += end_milli - start_milli;

            if(VERBOSE_MODE)
                std::cout << "Elapsed time [ms]: " << end_milli - start_milli << std::endl << std::endl;




        	if(VISUALISE_MODE) {
                for(size_t i = 0; i < haar_face_features.size(); i++) {
                    rectangle(rgbimage, haar_face_features[i], BLUE, 2);
                }
                for(size_t i = 0; i < hog_features.size(); i++) {
                    rectangle(rgbimage, hog_features[i], GREEN, 2);
                }
                rectangle(rgbimage, pc_feature, RED, 2);
                ellipse(rgbimage, person_centroid, Size(5,5), 0, 0, 360, CYAN, -1, 8, 0 );



                putText(rgbimage, "Haar face", Point(20,20), FONT_HERSHEY_PLAIN, 1.2, BLUE);
                putText(rgbimage, "HoG", Point(20,40), FONT_HERSHEY_PLAIN, 1.2, GREEN);
                putText(rgbimage, "Depth", Point(20,60), FONT_HERSHEY_PLAIN, 1.2, RED);
                putText(rgbimage, "Result", Point(20,80), FONT_HERSHEY_PLAIN, 1.2, CYAN);


                namedWindow("Persontracking", WINDOW_AUTOSIZE);
                imshow("Persontracking", rgbimage);
                waitKey(1);
                viewer->addCube(min_point.x, max_point.x, min_point.y, max_point.y, min_point.z, max_point.z, 1.0, 0.0, 0.0);
                viewer->addPointCloud(cloud, cloud_color_handler, "cloud");
                viewer->spinOnce();
                viewer->removeAllPointClouds();
                viewer->removeAllShapes();
            }



            ++rgb_iterator;
            ++depth_iterator;


        } //end cycling through frames



        float avr_time_per_frame = time_accumulator/NUM_FRAMES;
        if(VERBOSE_MODE) {
            std::cout << "Frame rate [Hz]: " << 1000.0f/avr_time_per_frame << std::endl << std::endl;
        }
        times[run] = 1000.0f/avr_time_per_frame;


        rgb_iterator = rgb_path_vec.begin();
        depth_iterator = depth_path_vec.begin();
        time_accumulator = 0;

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
		std::cout << "Pop. Standard Deviation: " << stats.std_dev << " [ms]" << std::endl << std::endl;
	}

	//write results to file
	if(VERBOSE_MODE)
		std::cout << "Writing Results to File." << std::endl << std::endl;

		results_file << "#Results from Person Tracking workload\n";
		results_file << "Metric: time to detect a person in a frame\n";
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


    std::cout << "Ending Person Tracking Workload" << std::endl << std::endl;


    return 0;
}
