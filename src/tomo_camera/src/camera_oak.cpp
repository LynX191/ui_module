/// Oak camera interface class; Based on old version, cleaned up
/// Udupa; Nov'2022

#include <ros/ros.h>

#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgcodecs/imgcodecs.hpp>
#include <opencv2/imgproc/imgproc.hpp>

#include "tomo_camera/camera_oak.h"

using namespace cv;

CameraOak::CameraOak() : camera_id(""), camera_mode(OAK_CAMERA_MODE::NO_CAMERA)
{
}

CameraOak::~CameraOak()
{
	close();
}

std::shared_ptr<dai::Pipeline> CameraOak::createPipeline()
{
	auto pipeline = std::make_shared<dai::Pipeline>();
	// Create pipeline
	// Define source and output
	auto camRgb	   = pipeline->create<dai::node::ColorCamera>();
	auto xoutVideo = pipeline->create<dai::node::XLinkOut>();
	auto controlIn = pipeline->create<dai::node::XLinkIn>();

	xoutVideo->setStreamName("video");
	controlIn->setStreamName("control");

	// Properties
	camRgb->setBoardSocket(dai::CameraBoardSocket::RGB);
	if(camera_mode == OAK_OPTISPEC) {
		camRgb->setResolution(dai::ColorCameraProperties::SensorResolution::THE_48_MP);
		camRgb->setVideoSize(5312, 6000);
	}
	else if(camera_mode == OAK_HEAD) {
		camRgb->setResolution(dai::ColorCameraProperties::SensorResolution::THE_1080_P);
		camRgb->setVideoSize(1920, 1080);
	}
	else {
		camRgb->setResolution(dai::ColorCameraProperties::SensorResolution::THE_4_K);
		camRgb->setVideoSize(3840, 2160);
	}

	// xoutVideo->input.setBlocking(false);
	// xoutVideo->input.setQueueSize(max_queue_size);

	// Linking
	camRgb->isp.link(xoutVideo->input);
	controlIn->out.link(camRgb->inputControl);

	return pipeline;
}

bool CameraOak::registerCamera(int cameraMode)
{
	if(cameraMode == NO_CAMERA)
		return false;
	if(cameraMode == OAK_OPTISPEC)
		return registerCamera(OPTISPEC_CAM_ID, OAK_OPTISPEC, 1);
	if(cameraMode == OAK_BOM)
		return registerCamera(BOM_CAM_ID, OAK_BOM, 1);
	if(cameraMode == OAK_HEAD)
		return registerCamera(HEAD_CAM_ID, OAK_HEAD, 1);
};

bool CameraOak::registerCamera(string cameraId, int cameraMode, int maxQueueSize)
{
	camera_id = cameraId;

	camera_mode = cameraMode;
	try {
		ROS_INFO("[OakCam %s] Registering camera...", camera_id.c_str());

		auto deviceDesc = dai::Device::getDeviceByMxId(camera_id.c_str());
		if(!std::get<0>(deviceDesc)) {
			ROS_ERROR("[OakCam %s] Camera not found", camera_id.c_str());
			return false;
		}

		auto devInfo	= std::get<1>(deviceDesc);
		camera_pipeline = createPipeline();
		camera_device	= std::make_shared<dai::Device>(*camera_pipeline, devInfo, dai::UsbSpeed::SUPER);
		control_queue	= camera_device->getInputQueue("control");
		video_output	= camera_device->getOutputQueue("video", maxQueueSize, false);

		// Setting Camera Control
		dai::CameraControl camControl;
		if(cameraMode == OAK_OPTISPEC) {
			camControl.setAutoWhiteBalanceMode(dai::CameraControl::AutoWhiteBalanceMode::OFF);
			camControl.setManualFocus(137);
			camControl.setManualExposure(25000, 300);
			camControl.setManualWhiteBalance(5200);
		}
		else {
			camControl.setManualFocus(135);
			camControl.setManualExposure(30000, 120);
		}
		control_queue->send(camControl);
		ROS_WARN("[OakCam %s] Camera registered", camera_id.c_str());
	}
	catch(const std::exception& e) {
		ROS_ERROR("[OakCam %s] Error while registering camera (%s)", camera_id.c_str(), e.what());
		return false;
	}

	return true;
}

bool CameraOak::isOpen()
{
	return camera_device && video_output && !camera_device->isClosed();
}

bool CameraOak::getFrame(Mat& cvFrame, int timeout)
{
	if(!isOpen()) {
		if(!registerCamera(camera_mode)) {
			ROS_ERROR("[OakCam %s] getFrame: Camera not open", camera_id.c_str());
			return false;
		}
	}

	while(isOpen()) {
		try {
			ROS_INFO("[OakCam %s] Fetching frame from camera", camera_id.c_str());
			auto startTime = std::chrono::high_resolution_clock::now();
			auto videoIn   = video_output->get<dai::ImgFrame>();
			ROS_INFO("[OakCam %s] Image frame handle fetched", camera_id.c_str());
			cvFrame = videoIn->getCvFrame();
			if(!cvFrame.empty()) {
				std::chrono::duration<double> elapsed = std::chrono::high_resolution_clock::now() - startTime;
				ROS_INFO("[OakCam %s] Frame received (%d ms)", camera_id.c_str(), (int) (elapsed.count() * 1000));
				break;
			}
		}
		catch(std::exception& e) {
			ROS_ERROR("[OakCam %s] getFrame exception (%s)", camera_id.c_str(), e.what());
			usleep(5000);
			continue;
		}
	}

	return !cvFrame.empty();
}

void CameraOak::close()
{
	if(camera_device != nullptr) {
		ROS_INFO("[OakCam %s] closing", camera_id.c_str());
		camera_device->close();
		ROS_WARN("[OakCam %s] closed", camera_id.c_str());
	}
}