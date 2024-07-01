#include "head_vm.h"
#include <QTimer>
#include <ros/ros.h>
HeadVM::HeadVM(MasterApp* masterApp, QObject* parent) : master_app(masterApp), QObject{parent}
{
	_isSuperUserAct = false;
	if(master_app) {
		tomoeye_station =
			std::make_shared<StationCore>(STATION_0, "tomo_eye", tVectorS({"Stream", "CartonVision|0", "ClosingCheck|0", "OutputCheck|0"}));
		master_app->stream_tracks[STATION_0] = tomoeye_station;
		master_app->track_status[STATION_0]	 = tomoeye_station->initialize();
		track_view_models.insert("TomoEye", master_app->track_view_models["TomoEye"]);
	}
	createConnection();
	setAutoSwitch(true);
}

HeadVM::~HeadVM()
{
}

int HeadVM::currentTabViewModel()
{
	return _currentTabViewModel;
}

void HeadVM::createConnection()
{
	connect(tomoeye_station.get(), SIGNAL(updateImage_Station_VM_Signal(int)), this, SLOT(refreshFrame(int)));
}

int HeadVM::getTabViewModel()
{
	return currentTabViewModel();
}

bool HeadVM::refreshImage(int docIndex, StreamData& streamData)
{
	if(!master_app->isSuperUser)
		return false;
	UI_INFO("[HeadVM::refreshImage] Displaying image doc #%d", docIndex);
	if(!streamData.q_image.isNull())
		track_view_models.value("TomoEye")->UpdateImage(streamData.q_image, docIndex);
	return true;
}

void HeadVM::refreshFrame(int docIndex)
{
	if(!master_app->isSuperUser)
		return;
	UI_INFO("[HeadVM::refreshFrame] View %d", docIndex);
	if(docIndex < 0)
		return;
	StreamData streamData;
	if(!tomoeye_station->getFrame(docIndex, streamData) || streamData.image_frame.empty()) {
		if(!streamData.image_frame.empty())
			UI_ERROR("[HeadVM::refreshFrame] Doc %d: Failed to fetch frame", docIndex);
		return;
	}
	if(_autoSwitch)
		Q_EMIT setTabViewIndex(docIndex);
	if(!refreshImage(docIndex, streamData)) {
		UI_ERROR("[HeadVM::refreshFrame %d] Failed to refresh frame", docIndex);
		return;
	}
}

void HeadVM::clickToolButton(QString trackName, QString btnName, bool state)
{
	UI_INFO("[HeadVM::clickToolButton] Track Name %s, Button Name %s", convertToStdString(trackName).c_str(), convertToStdString(btnName).c_str());
	if(!master_app->isSuperUser)
		return;
	if(trackName != "TomoEye")
		return;
	if(btnName == "auto_switch")
		setAutoSwitch(state, false);
	else if(btnName == "live") {
		tomoeye_station->startStreaming(state ? "camera" : "");
		Q_EMIT liveSwitchChanged(state);
		if(!state)
			return;
		setAutoSwitch(false);
		Q_EMIT setTabViewIndex(0);
	}
	else if(btnName == "export"){
		master_app->dashboard_vm->setListModel(false);
		master_app->dashboard_vm->setExpandDir(0);
	}
}

void HeadVM::setAutoSwitch(bool state, bool emit)
{
	if(_autoSwitch = state)
		return;
	_autoSwitch = state;
	if(!emit)
		return;
	Q_EMIT autoSwitchChanged(state);
}

void HeadVM::loadImageFromFile(QString path)
{
	return;
}

void HeadVM::tabViewChange(int index)
{
	if(index < 0)
		return;
	master_app->config.camsFocus.at(1) = cameraFocusToString[static_cast<CAMERAS_FOCUS>(index + 10)];
	master_app->config.writeFileConfig();
}

void HeadVM::tabIndexChange(int index)
{
	return;
}

void HeadVM::setAfterUiCreated()
{
	Q_EMIT autoSwitchChanged(_autoSwitch);
	// Q_EMIT setStateConnectionTomOImaging(true);
	QTimer::singleShot(1000, this, SLOT(delaySendSignal()));
}

void HeadVM::delaySendSignal()
{
	Q_EMIT setTabViewIndex(master_app->config.getCameraFocusFromString(master_app->config.camsFocus.at(1)) - 10);
}

void HeadVM::clearResult()
{
	if(!master_app->isSuperUser)
		return;
	QImage blackImage = QImage(100, 100, QImage::Format_RGB888);
	blackImage.fill(Qt::black);
	int index = 0;
	Q_FOREACH(QSharedPointer<TrackVM> value, track_view_models.values()) {
		value->UpdateImage(blackImage, index);
		index++;
	}
}

bool HeadVM::changeMxId(std::string mxId)
{
	tomoeye_station->changeMxIdCam(mxId);
}

void HeadVM::setSuperUserAct(bool value)
{
	_isSuperUserAct = value;
}
