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
#include <time.h>
#include <vector>
#include <memory>

#include <caffe/caffe.hpp>
#ifdef USE_OPENCV
	#include <opencv2/core/core.hpp>
	#include <opencv2/highgui/highgui.hpp>
	#include <opencv2/imgproc/imgproc.hpp>
#endif  // USE_OPENCV
#include <algorithm>
#include <iosfwd>
#include <string>
#include <utility>
#include <vector>

#include <Eigen/Geometry>

#include <pcl/common/transforms.h>
#include <pcl/point_cloud.h>
#include <pcl/filters/statistical_outlier_removal.h>
#include <pcl/io/pcd_io.h>
#include <pcl/visualization/pcl_visualizer.h>
#include <pcl/common/common.h>
#include <pcl/filters/passthrough.h>
#include <pcl/search/kdtree.h>
#include <pcl/segmentation/sac_segmentation.h>
#include <pcl/sample_consensus/sac_model_plane.h>
#include <pcl/filters/extract_indices.h>
#include <pcl/segmentation/extract_clusters.h>


#include "botmarkcommon.h"

using namespace caffe;
using std::string;



const float KINECT_FOCAL_LENGTH = 575.8157496f;
const cv::Scalar RED(0, 0, 255);
const cv::Scalar BLUE(255, 0, 0);
const float APRON = 1.1f;


/* Pair (label, confidence) representing a prediction. */
typedef std::pair<string, float> Prediction;
typedef std::vector<boost::filesystem::path> path_vector;


struct timespec start_time;
struct timespec end_time;
float time_accumulator=0.0f;





class Classifier {
	public:
		Classifier(const string& model_file,
			const string& trained_file,
			const string& mean_file,
			const string& label_file);

		std::vector<Prediction> Classify(const cv::Mat& img, int N = 5);

	private:
		void SetMean(const string& mean_file);

		std::vector<float> Predict(const cv::Mat& img);

		void WrapInputLayer(std::vector<cv::Mat>* input_channels);

		void Preprocess(const cv::Mat& img,
		std::vector<cv::Mat>* input_channels);

	private:
		shared_ptr<Net<float> > net_;
		cv::Size input_geometry_;
		int num_channels_;
		cv::Mat mean_;
		std::vector<string> labels_;
};

Classifier::Classifier(const string& model_file,
	const string& trained_file,
	const string& mean_file,
	const string& label_file) {

	Caffe::set_mode(Caffe::CPU);


	/* Load the network. */
	net_.reset(new Net<float>(model_file, TEST));
	net_->CopyTrainedLayersFrom(trained_file);

	CHECK_EQ(net_->num_inputs(), 1) << "Network should have exactly one input.";
	CHECK_EQ(net_->num_outputs(), 1) << "Network should have exactly one output.";

	Blob<float>* input_layer = net_->input_blobs()[0];
	num_channels_ = input_layer->channels();
	CHECK(num_channels_ == 3 || num_channels_ == 1)
	<< "Input layer should have 1 or 3 channels.";
	input_geometry_ = cv::Size(input_layer->width(), input_layer->height());

	/* Load the binaryproto mean file. */
	SetMean(mean_file);

	/* Load labels. */
	std::ifstream labels(label_file.c_str());
	CHECK(labels) << "Unable to open labels file " << label_file;
	string line;
	while (std::getline(labels, line))
	labels_.push_back(string(line));

	Blob<float>* output_layer = net_->output_blobs()[0];
	CHECK_EQ(labels_.size(), output_layer->channels())
	<< "Number of labels is different from the output layer dimension.";
}

static bool PairCompare(const std::pair<float, int>& lhs,
const std::pair<float, int>& rhs) {
	return lhs.first > rhs.first;
}

/* Return the indices of the top N values of vector v. */
static std::vector<int> Argmax(const std::vector<float>& v, int N) {
	std::vector<std::pair<float, int> > pairs;
	for (size_t i = 0; i < v.size(); ++i)
		pairs.push_back(std::make_pair(v[i], i));
	std::partial_sort(pairs.begin(), pairs.begin() + N, pairs.end(), PairCompare);

	std::vector<int> result;
	for (int i = 0; i < N; ++i)
		result.push_back(pairs[i].second);
	return result;
}

/* Return the top N predictions. */
std::vector<Prediction> Classifier::Classify(const cv::Mat& img, int N) {
	std::vector<float> output = Predict(img);

	N = std::min<int>(labels_.size(), N);
	std::vector<int> maxN = Argmax(output, N);
	std::vector<Prediction> predictions;
	for (int i = 0; i < N; ++i) {
		int idx = maxN[i];
		predictions.push_back(std::make_pair(labels_[idx], output[idx]));
	}

	return predictions;
}

/* Load the mean file in binaryproto format. */
void Classifier::SetMean(const string& mean_file) {
	BlobProto blob_proto;
	ReadProtoFromBinaryFileOrDie(mean_file.c_str(), &blob_proto);

	/* Convert from BlobProto to Blob<float> */
	Blob<float> mean_blob;
	mean_blob.FromProto(blob_proto);
	CHECK_EQ(mean_blob.channels(), num_channels_)
	<< "Number of channels of mean file doesn't match input layer.";

	/* The format of the mean file is planar 32-bit float BGR or grayscale. */
	std::vector<cv::Mat> channels;
	float* data = mean_blob.mutable_cpu_data();
	for (int i = 0; i < num_channels_; ++i) {
		/* Extract an individual channel. */
		cv::Mat channel(mean_blob.height(), mean_blob.width(), CV_32FC1, data);
		channels.push_back(channel);
		data += mean_blob.height() * mean_blob.width();
	}

	/* Merge the separate channels into a single image. */
	cv::Mat mean;
	cv::merge(channels, mean);

	/* Compute the global mean pixel value and create a mean image
	* filled with this value. */
	cv::Scalar channel_mean = cv::mean(mean);
	mean_ = cv::Mat(input_geometry_, mean.type(), channel_mean);
}

std::vector<float> Classifier::Predict(const cv::Mat& img) {
	Blob<float>* input_layer = net_->input_blobs()[0];
	input_layer->Reshape(1, num_channels_,
	input_geometry_.height, input_geometry_.width);
	/* Forward dimension change to all layers. */
	net_->Reshape();

	std::vector<cv::Mat> input_channels;
	WrapInputLayer(&input_channels);

	Preprocess(img, &input_channels);

	net_->Forward();

	/* Copy the output layer to a std::vector */
	Blob<float>* output_layer = net_->output_blobs()[0];
	const float* begin = output_layer->cpu_data();
	const float* end = begin + output_layer->channels();
	return std::vector<float>(begin, end);
}

void Classifier::WrapInputLayer(std::vector<cv::Mat>* input_channels) {
	Blob<float>* input_layer = net_->input_blobs()[0];

	int width = input_layer->width();
	int height = input_layer->height();
	float* input_data = input_layer->mutable_cpu_data();
	for (int i = 0; i < input_layer->channels(); ++i) {
		cv::Mat channel(height, width, CV_32FC1, input_data);
		input_channels->push_back(channel);
		input_data += width * height;
	}
}

void Classifier::Preprocess(const cv::Mat& img,
std::vector<cv::Mat>* input_channels) {
	/* Convert the input image to the input image format of the network. */
	cv::Mat sample;
	if (img.channels() == 3 && num_channels_ == 1)
	cv::cvtColor(img, sample, cv::COLOR_BGR2GRAY);
	else if (img.channels() == 4 && num_channels_ == 1)
	cv::cvtColor(img, sample, cv::COLOR_BGRA2GRAY);
	else if (img.channels() == 4 && num_channels_ == 3)
	cv::cvtColor(img, sample, cv::COLOR_BGRA2BGR);
	else if (img.channels() == 1 && num_channels_ == 3)
	cv::cvtColor(img, sample, cv::COLOR_GRAY2BGR);
	else
	sample = img;

	cv::Mat sample_resized;
	if (sample.size() != input_geometry_)
	cv::resize(sample, sample_resized, input_geometry_);
	else
	sample_resized = sample;

	cv::Mat sample_float;
	if (num_channels_ == 3)
	sample_resized.convertTo(sample_float, CV_32FC3);
	else
	sample_resized.convertTo(sample_float, CV_32FC1);

	cv::Mat sample_normalized;
	cv::subtract(sample_float, mean_, sample_normalized);

	cv::split(sample_normalized, *input_channels);

	CHECK(reinterpret_cast<float*>(input_channels->at(0).data)
	== net_->input_blobs()[0]->cpu_data())
	<< "Input channels are not wrapping the input layer of the network.";
}








int main(int argc, char** argv) {

	std::cout << "Botmark version: " << Botmark_VERSION_MAJOR << "." <<
	Botmark_VERSION_MINOR << "." << Botmark_VERSION_PATCH << std::endl;
	if(BENCHMARK_RUNS < 4) {
		std::cout << "[Botmark Error] Number of runs is less than 4, please change in botmarkcommon.h" << std::endl << std::endl;
		return -1;
	}

	std::cout << "Starting Object Recognition Workload" << std::endl << std::endl;

	std::ofstream results_file;
	results_file.open(RESULTS_FILE.c_str(), std::ofstream::app);
	if(!results_file) {
		std::cout << "[Botmark Error] Failed to open results file." << std::endl;
		return -1;
	}
	double times[BENCHMARK_RUNS];







  //init
	//disable logging - Caffe is pretty chatty
	::google::InitGoogleLogging(argv[0]);

	//initialise directory iterators
	path_vector rgb_path_vec, pc_path_vec;
	copy(boost::filesystem::directory_iterator(OBJECT_RECOGNITION_DATA_PATH+"rgb/"),
		boost::filesystem::directory_iterator(),
		back_inserter(rgb_path_vec));
	copy(boost::filesystem::directory_iterator(OBJECT_RECOGNITION_DATA_PATH+"point_clouds/"),
		boost::filesystem::directory_iterator(),
		back_inserter(pc_path_vec));
	path_vector::const_iterator rgb_iterator(rgb_path_vec.begin());
	path_vector::const_iterator pc_iterator(pc_path_vec.begin());
	sort(rgb_path_vec.begin(), rgb_path_vec.end());
	sort(pc_path_vec.begin(), pc_path_vec.end());


	pcl::PointCloud<pcl::PointXYZ>::Ptr original_cloud(new pcl::PointCloud<pcl::PointXYZ> ()),
		two_cloud(new pcl::PointCloud<pcl::PointXYZ>),
		final_cloud(new pcl::PointCloud<pcl::PointXYZ>);
	pcl::PCDReader reader;

	boost::shared_ptr<pcl::visualization::PCLVisualizer> viewer;
	if(VISUALISE_MODE) {
		viewer = boost::make_shared<pcl::visualization::PCLVisualizer>("Object Recognition Viewer");
		viewer->setBackgroundColor(0.05, 0.05, 0.05, 0);
		viewer->setCameraPosition(0, -0.1, -2.5, 0, -1, 0);
	}


	Classifier classifier(CAFFE_MODEL_FILE, CAFFE_TRAINED_FILE, CAFFE_MEAN_FILE, CAFFE_LABEL_FILE);


	//start benchmarking runs
	for(int run=0; run<BENCHMARK_RUNS; run++) {

		if(VERBOSE_MODE)
			std::cout << "Run: " << run+1 << " of " << BENCHMARK_RUNS << std::endl;

		//cycle over frames
		for(int i=0; i<OBJECT_RECOGNITION_FRAMES; i++) {


			// read in data
			if(reader.read<pcl::PointXYZ> ((*pc_iterator).string(), *original_cloud) != 0) {
				std::cout << "[Botmark Error] Failed to open file." << std::endl;
				return -1;
			}

			string rgb_filename = (*rgb_iterator).string();
			cv::Mat img = cv::imread(rgb_filename, cv::IMREAD_COLOR);



			//start clock
			if(clock_gettime(CLOCK_MONOTONIC_RAW, &start_time)) {
				std::cout << "[Botmark Error] Can't use clock timing routine." << std::endl;
				return -1;
			}


			//pass through filter on pc
			pcl::PassThrough<pcl::PointXYZ> pass;
			pass.setFilterFieldName("z");
			pass.setFilterLimits(0.0, 1.2);
			pass.setInputCloud(original_cloud);
			pass.filter(*two_cloud);


			//remove most 'prominent' plane
			pcl::ModelCoefficients::Ptr coefficients(new pcl::ModelCoefficients);
			pcl::SACSegmentation<pcl::PointXYZ> segmentation;
			segmentation.setInputCloud(two_cloud);
			segmentation.setModelType(pcl::SACMODEL_PLANE);
			segmentation.setMethodType(pcl::SAC_RANSAC);
			segmentation.setDistanceThreshold(0.015);
			segmentation.setOptimizeCoefficients(true);
			pcl::PointIndices::Ptr planeIndices(new pcl::PointIndices);
			segmentation.segment(*planeIndices, *coefficients);

			if(planeIndices->indices.size() == 0) {
				std::cout << "[Botmark Error] Could not find a plane in the scene." << std::endl;
			} else {
				// Copy the points of the plane to a new cloud.
				pcl::ExtractIndices<pcl::PointXYZ> extract;
				extract.setInputCloud(two_cloud);
				extract.setNegative(true);
				extract.setIndices(planeIndices);
				extract.filter(*final_cloud);
			}



			// cluster remaining pc
			pcl::search::KdTree<pcl::PointXYZ>::Ptr tree(new pcl::search::KdTree<pcl::PointXYZ>);
			tree->setInputCloud(final_cloud);

			std::vector<pcl::PointIndices> cluster_indices;
			pcl::EuclideanClusterExtraction<pcl::PointXYZ> ec;
			ec.setClusterTolerance(0.04);
			ec.setMinClusterSize(100);
			ec.setMaxClusterSize(25000);
			ec.setSearchMethod(tree);
			ec.setInputCloud(final_cloud);
			ec.extract(cluster_indices);


			int centerX = img.cols/2;
			int centerY = img.rows/2;

			//cycle over clusters
			std::vector<cv::Rect> object_rects;
			int j = 0;
			for(std::vector<pcl::PointIndices>::const_iterator it = cluster_indices.begin (); it != cluster_indices.end (); ++it) {

				pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_cluster(new pcl::PointCloud<pcl::PointXYZ>);

				for (std::vector<int>::const_iterator pit = it->indices.begin (); pit != it->indices.end (); ++pit) {
					cloud_cluster->points.push_back(final_cloud->points[*pit]);
				}


				cloud_cluster->width = cloud_cluster->points.size();
				cloud_cluster->height = 1;
				cloud_cluster->is_dense = true;


				std::stringstream stream;
				stream << j;
				std::string shape_name = "shape_" + stream.str();


				pcl::PointXYZ proj_min;
				pcl::PointXYZ proj_max;
				pcl::getMinMax3D(*cloud_cluster, proj_min, proj_max);
				if(proj_min.x > 0) {
					proj_min.z = proj_max.z;
					proj_max.z = proj_min.z;
				}


				Eigen::Vector4f centroid;
				pcl::compute3DCentroid(*cloud_cluster, centroid);
				Eigen::Vector3f translation = centroid.head<3>();
				Eigen::Quaternionf rotation(0, 0, 0, 0);

				if(VISUALISE_MODE) {
					viewer->addCube(translation, rotation, proj_max.x-proj_min.x, proj_max.y-proj_min.y, proj_max.z-proj_min.z, shape_name);
				}



				object_rects.push_back(cv::Rect(
					cv::Point((proj_max.x)*(KINECT_FOCAL_LENGTH/proj_max.z)+centerX, (proj_max.y)*(KINECT_FOCAL_LENGTH/proj_max.z)+centerY),
					cv::Point((proj_min.x)*(KINECT_FOCAL_LENGTH/proj_min.z)+centerX, (proj_min.y)*(KINECT_FOCAL_LENGTH/proj_min.z)+centerY))
					);



				//visualisation
				if(VISUALISE_MODE) {
					pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZ> cluster_cloud_color_handler (cloud_cluster, 100 + (std::rand() % (255 - 100 + 1)), 100 + (std::rand() % (255 - 100 + 1)), 100 + (std::rand() % (255 - 100 + 1)));

					//std::stringstream stream;
					stream << j;
					std::string name = "cloud_" + stream.str();
					viewer->addPointCloud(cloud_cluster, cluster_cloud_color_handler, name);
					viewer->setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 1, name);

				}


				j++;
			} // loop over clusters


			for(size_t i = 0; i < object_rects.size(); i++) {
				cv::Size deltaSize(object_rects[i].width*APRON, object_rects[i].height*APRON );
				cv::Point offset(deltaSize.width/2, deltaSize.height/2);
				object_rects[i] += deltaSize;
				object_rects[i] -= offset;

				object_rects[i] &= cv::Rect(cv::Point(0, 0), img.size()); //get intersection to ensure the r.o.i. is in bounds

			}


			/*** caffe ***/
			for(size_t i = 0; i < object_rects.size(); i++) {
				cv::Mat this_object_image = img(object_rects[i]);

				CHECK(!this_object_image.empty()) << "Error decoding image " << rgb_filename;
				std::vector<Prediction> predictions = classifier.Classify(this_object_image);

				Prediction p = predictions[0];
				if(VERBOSE_MODE) {
						std::cout << "predicted: " << std::fixed << std::setprecision(4) << p.second << " - \"" << p.first << "\"" << std::endl;
				}


				if(VISUALISE_MODE) {
					cv::rectangle(img, object_rects[i], RED, 2);
					cv::putText(img, p.first, cv::Point(object_rects[i].x-10, object_rects[i].y-10), cv::FONT_HERSHEY_PLAIN, 1.0, RED);
				}
			}

			//stop clock
			if(clock_gettime(CLOCK_MONOTONIC_RAW, &end_time)) {
				std::cout << "[Botmark Error] Can't use clock timing routine." << std::endl;
				return -1;
			}
			//calculate time
			double start_milli = (double) 1.0e3*start_time.tv_sec + 1.0e-6*start_time.tv_nsec;
			double end_milli = (double) 1.0e3*end_time.tv_sec + 1.0e-6*end_time.tv_nsec;
			time_accumulator += end_milli - start_milli;



			if(VERBOSE_MODE) {
					std::cout << "Elapsed time [ms]: " << end_milli - start_milli << std::endl << std::endl;
			}



			if(VISUALISE_MODE) {
				viewer->spinOnce();
				cv::namedWindow("Recognised objects", cv::WINDOW_AUTOSIZE);
				cv::imshow("Recognised objects", img);
				cv::waitKey(1);
				viewer->removeAllPointClouds();
				viewer->removeAllShapes();
			}


	    ++rgb_iterator;
	    ++pc_iterator;

		} // end loop over frames


    float avr_time_per_frame = time_accumulator/OBJECT_RECOGNITION_FRAMES;
    if(VERBOSE_MODE) {
        std::cout << "Frame rate [Hz]: " << 1000.0f/avr_time_per_frame << std::endl << std::endl;
    }
    times[run] = 1000.0f/avr_time_per_frame;


    rgb_iterator = rgb_path_vec.begin();
    pc_iterator = pc_path_vec.begin();
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
		std::cout << "Pop. Standard Deviation: " << stats.std_dev << " [ms]" << std::endl;
		std::cout << "Writing Results to File." << std::endl << std::endl;
	}

	//write results to file
	results_file << "#Results from Object Recognition workload\n";
	results_file << "Metric: time to recognise objects in a frame\n";
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


	std::cout << "Ending Object Recognition Workload" << std::endl << std::endl;






	return 0;
}
