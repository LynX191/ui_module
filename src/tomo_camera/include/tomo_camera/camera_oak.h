/// Oak camera interface class; Based on old version, cleaned up
/// Udupa; Nov'2022

#pragma once

#include <iostream>
#include <opencv2/opencv.hpp>

#include "depthai/depthai.hpp"

/// Temporarily creating camera mode until the parameters are read from file; Udupa; Nov'2022
enum OAK_CAMERA_MODE
{
	OAK_OPTISPEC,
	OAK_BOM,
	OAK_HEAD,
	NO_CAMERA
};
const static std::string OPTISPEC_CAM_ID = "18443010F13CF50800";
const static std::string BOM_CAM_ID		 = "184430108165940F00";
const static std::string HEAD_CAM_ID	 = "18443010611D631200";

using namespace std;

class CameraOak
{
public:
	CameraOak();
	~CameraOak();

public:
	bool registerCamera(int cameraMode);
	bool registerCamera(string cameraId, int cameraMode, int maxQueueSize = 1);
	bool getFrame(cv::Mat& cvFrame, int timeout = -1);
	bool isOpen();
	void close();

private:
	string camera_id;
	int camera_mode;

	std::shared_ptr<dai::DataInputQueue> control_queue;
	std::shared_ptr<dai::DataOutputQueue> video_output;
	std::shared_ptr<dai::Pipeline> camera_pipeline;
	std::shared_ptr<dai::Device> camera_device;

	std::shared_ptr<dai::Pipeline> createPipeline();
};