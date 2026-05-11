#pragma once

#include "parameters.h"
#include "keyframe.h"

// ROS2 includes for LoopDetector
#include <rclcpp/rclcpp.hpp>

using namespace DVision;
using namespace DBoW2;

class LoopDetector
{
public:

	BriefDatabase db;
	BriefVocabulary* voc;

	map<int, cv::Mat> image_pool;

	list<KeyFrame*> keyframelist;

	// ROS2 node for logging and publisher access
	static rclcpp::Node::SharedPtr node_;

	LoopDetector();
	void loadVocabulary(std::string voc_path);
	
	void addKeyFrame(KeyFrame* cur_kf, bool flag_detect_loop);
	void addKeyFrameIntoVoc(KeyFrame* keyframe);
	KeyFrame* getKeyFrame(int index);

	void visualizeKeyPoses(double time_cur);

	int detectLoop(KeyFrame* keyframe, int frame_index);

	// Static method to set ROS2 node
	static void setNode(rclcpp::Node::SharedPtr node) { node_ = node; }
};