// Base class for station/document class hierarchy to manage video/image stream data, inspection and result handling
// Udupa; Oct/Nov'2022
// Restructured for server based image streaming; Udupa; Nov'2023

#include "station_core.h"

StationCore::StationCore(int cameraIndex, std::string stationName, tVectorS docNames) : camera_index(cameraIndex), station_name(stationName)
{
	stream_source = "";
	int index	  = 0;
	for(auto& name : docNames)
		doc_indices[name] = index++;

	live_mode = false;
}

StationCore::~StationCore()
{
}

bool StationCore::initialize()
{
	std::string cameraIndex = std::to_string(camera_index);
	if(!TopicImage::initSubscriber("/tomo_imaging/stream_ui_" + cameraIndex))
		return false;

	if(!imaging_comm.initPublisher("/tomo_ui/comm_imaging_" + cameraIndex))
		return false;

	return true;
}

bool StationCore::startStreaming(const std::string& streamSource)
{
	stream_source = streamSource;
	bool start	  = !streamSource.empty();
	if(start && streamSource.find("camera"))
		// loadImageFromFile(0);
		;
	else {
		live_mode = !live_mode;	 // Until UI element is fixed
		start	  = live_mode;	 // Until UI element is fixed
		sendCommand("set_stream_" + std::string(start ? "on" : "off"));
	}
}
bool StationCore::changeMxIdCam(std::string& command)
{
	sendCommand(" change_came_" + command);
}

void StationCore::sendCommand(const std::string& command)
{
	imaging_comm.sendMessage(command);
}

void StationCore::processReceivedData()
{
	if(!received_image.id.rfind("algo_list", 0)) {}
	else {
		if(!received_image.image.empty()) {
			int docIndex = 0;
			if(doc_indices.count(received_image.id))
				docIndex = doc_indices[received_image.id];
			else
				ROS_ERROR("Invalid doc index from imaging server; using doc 0");
			setFrame(docIndex, received_image.image);
			updateImage(docIndex);
		}
		else
			ROS_ERROR("[StationCore.processReceivedImage] Empty image received");
	}
}

bool StationCore::setFrame(int docIndex, const cv::Mat& imageMat)
{
	stream_data[docIndex] = StreamData(imageMat, MatToQImage(imageMat));
}

bool StationCore::getFrame(int docIndex, StreamData& streamData)
{
	streamData = stream_data[docIndex];
	return true;
}

void StationCore::updateImage(int docIndex)
{
	Q_EMIT updateImage_Station_VM_Signal(docIndex);
}

// void StationCore::loadImageFromFile(int docIndex)
// {
// 	if(stream_source.empty()) {
// 		ROS_ERROR("Stream: Invalid image path");
// 		return;
// 	}

// 	ROS_WARN("Stream: loadImageFromFile from file %s", stream_source.c_str());
// 	cv::Mat cvImage;
// 	cvImage = cv::imread(stream_source);
// 	if(!cvImage.empty()) {
// 		setFrame(docIndex, cvImage);
// 		updateImage(docIndex);
// 	}
// 	ROS_WARN("Stream: Image file %s loaded\n\n\n", stream_source.c_str());
// }

void StationCore::saveImageToFile(int docIndex, bool includeTime)
{
	std::string filePath =
		getenv("HOME") + std::string("/tomo_stats/dataset/") + (includeTime ? getCurrentTime(true, false) : "") + station_name;
	filePath += ".png";
	cv::imwrite(filePath, stream_data[docIndex].image_frame);
	ROS_WARN("Stream: Stream saved to %s", filePath.c_str());
}

// From old version
QImage StationCore::MatToQImage(const cv::Mat& mat)
{
	switch(mat.type()) {
	// 8-bit, 4 channel
	case CV_8UC4: {
		QImage image(mat.data, mat.cols, mat.rows, static_cast<int>(mat.step), QImage::Format_ARGB32);

		return image;
	}

	// 8-bit, 3 channel
	case CV_8UC3: {
		QImage image(mat.data, mat.cols, mat.rows, static_cast<int>(mat.step), QImage::Format_RGB888);

		return image.rgbSwapped();
	}

	// 8-bit, 1 channel
	case CV_8UC1: {
#if QT_VERSION >= QT_VERSION_CHECK(5, 5, 0)
		QImage image(mat.data, mat.cols, mat.rows, static_cast<int>(mat.step), QImage::Format_Grayscale8);
#else
		static QVector<QRgb> sColorTable;

		// only create our color table the first time
		if(sColorTable.isEmpty()) {
			sColorTable.resize(256);

			for(int i = 0; i < 256; ++i) {
				sColorTable[i] = qRgb(i, i, i);
			}
		}

		QImage image(mat.data, mat.cols, mat.rows, static_cast<int>(mat.step), QImage::Format_Indexed8);

		image.setColorTable(sColorTable);
#endif
		return image;
	}
	default:
		ROS_WARN("ASM::cvMatToQImage() - cv::Mat image type not handled in switch: %d", mat.type());
		break;
	}
	return QImage();
}
