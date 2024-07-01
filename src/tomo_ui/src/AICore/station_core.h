// Base class for station/document class hierarchy to manage video/image stream data, inspection and result handling
// Udupa; Oct/Nov'2022
// Restructured for server based image streaming; Udupa; Nov'2023

#pragma once

#include <iostream>
#include <ros/ros.h>

#include <boost/scoped_array.hpp>
#include <boost/thread.hpp>
#include <boost/thread/mutex.hpp>

#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgcodecs/imgcodecs.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/opencv.hpp>

#include <QImage>
#include <QObject>

#include <tomo_comm/topic_image.h>
#include <tomo_comm/topic_string.h>

#include <tomo_utils/tomo_utils.h>

class StreamData
{
public:
	cv::Mat image_frame;
	QImage q_image;
	std::string insp_result;
	int insp_time;

	StreamData()
	{
	}

	StreamData(const cv::Mat& imageFrame, const QImage& qImage)
	{
		image_frame = imageFrame;
		q_image		= qImage;
	}

	void setResult(std::string inspResult, int inspTime)
	{
		insp_result = inspResult;
		insp_time	= inspTime;
	}

	void setResult(int inspTime)
	{
		insp_time = inspTime;
	}

	void clearResult()
	{
		insp_time	= 0;
		insp_result = "";
	}
};

using namespace std;
class StationCore : public QObject, public TopicImage
{
	Q_OBJECT

public:
	StationCore(int cameraIndex, std::string stationName, tVectorS docNames);
	~StationCore();

	bool initialize();
	bool startStreaming(const std::string& streamSource = "");
	bool getFrame(int docIndex, StreamData& streamData);
	bool changeMxIdCam(std::string& command);

protected:
	int camera_index;
	bool live_mode;
	std::map<int, StreamData> stream_data;
	string stream_source;

	std::string station_name;
	std::map<std::string, int> doc_indices;

	TopicString imaging_comm;

	void sendCommand(const std::string& command);
	void processReceivedData();

	bool setFrame(int docIndex, const cv::Mat& imageMat);
	virtual void updateImage(int docIndex);

	void loadImageFromFile(int docIndex);
	void saveImageToFile(int docIndex, bool includeTime = true);

	static QImage MatToQImage(const cv::Mat& mat);

Q_SIGNALS:
	void updateImage_Station_VM_Signal(int);
};
