/// Oak camera test function; Based on old qt version, streamlined
/// Udupa; Nov'2022

#include <ros/ros.h>

#include <tomo_camera/camera_oak.h>

bool isQuitApp(int key)
{
	return key == 'q' || key == 'Q';
}

int main(int argc, char* argv[])
{
	CameraOak cameraOak;
	cameraOak.registerCamera(HEAD_CAM_ID, OAK_HEAD, 1);
	// cameraOak.registerCamera(HEAD_CAM_ID);
	if(!cameraOak.isOpen()) {
		std::cout << "Camera not Open";
		return -1;
	}

	cv::Size imageSize(500, 600);
	// cv::VideoWriter videoWriter("/home/tomo4/ws_ts/video_output.avi", cv::VideoWriter::fourcc('X', 'V', 'I', 'D'), 10, imageSize);

	cv::namedWindow("Stream", cv::WINDOW_NORMAL);
	cv::resizeWindow("Stream", imageSize.width / 2, imageSize.height / 2);
	int count	   = 0;
	auto startTime = std::chrono::high_resolution_clock::now();
	auto curTime   = std::chrono::high_resolution_clock::now();

	bool saveVideo = false;
	while(true) {
		cv::Mat cvFrame;
		cameraOak.getFrame(cvFrame);
		cv::imshow("Stream", cvFrame);
		int key = cv::waitKey(1);
		if(isQuitApp(key))
			return 0;
		if(key == 's') {
			// cv::imwrite("/home/tomo4/ws_ts/capture" + std::to_string(count + 1) + ".png", cvFrame);
			count++;
		}
		if(key == 'v')
			saveVideo = !saveVideo;
		if(saveVideo)
			// videoWriter.write(cvFrame);
			curTime = std::chrono::high_resolution_clock::now();
		std::chrono::duration<double> elapsed = curTime - startTime;
		if(saveVideo)
			ROS_WARN("Cycle time: %d ms; Video saved", (int) (elapsed.count() * 1000));
		else
			ROS_INFO("Cycle time: %d ms", (int) (elapsed.count() * 1000));
		startTime = curTime;
	}
	// videoWriter.release();
	return 0;
}
