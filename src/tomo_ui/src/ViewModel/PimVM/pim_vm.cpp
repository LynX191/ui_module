#include "pim_vm.h"
#include <QTimer>
#include <ros/ros.h>

PimVM::PimVM(MasterApp* masterApp, QObject* parent) : master_app(masterApp), QObject{parent}
{
	_isSuperUserAct = false;
	if(master_app) {
		pim_station							 = std::make_shared<StationCore>(STATION_2, "pim", tVectorS({"Stream", "PimCartonCounting|0"}));
		master_app->stream_tracks[STATION_2] = pim_station;
		master_app->track_status[STATION_2]	 = pim_station->initialize();
		track_view_models.insert("PIM", master_app->track_view_models["PIM"]);
	}
	createConnection();
	setAutoSwitch(true);
}

PimVM::~PimVM()
{
}

int PimVM::currentTabViewModel()
{
	return _currentTabViewModel;
}

void PimVM::createConnection()
{
	connect(pim_station.get(), SIGNAL(updateImage_Station_VM_Signal(int)), this, SLOT(refreshFrame(int)));
}

int PimVM::getTabViewModel()
{
	return currentTabViewModel();
}

bool PimVM::refreshImage(int docIndex, StreamData& streamData)
{
	if(!master_app->isSuperUser)
		return false;
	UI_INFO("[HeadVM::refreshImage] Displaying image doc #%d", docIndex);
	if(!streamData.q_image.isNull())
		track_view_models.value("PIM")->UpdateImage(streamData.q_image, docIndex);
	return true;
}

void PimVM::refreshFrame(int docIndex)
{
	if(!master_app->isSuperUser)
		return;
	UI_INFO("[PimVM::refreshFrame] View %d", docIndex);
	if(docIndex < 0)
		return;
	StreamData streamData;
	if(!pim_station->getFrame(docIndex, streamData) || streamData.image_frame.empty()) {
		if(!streamData.image_frame.empty())
			UI_ERROR("[ShipperVM::refreshFrame] Doc %d: Failed to fetch frame", docIndex);
		return;
	}
	if(_autoSwitch)
		Q_EMIT setTabViewIndex(docIndex);
	if(!refreshImage(docIndex, streamData)) {
		UI_ERROR("[ShipperVM::refreshFrame %d] Failed to refresh frame", docIndex);
		return;
	}
}

void PimVM::clickToolButton(QString trackName, QString btnName, bool state)
{
	UI_INFO("[PimVM::clickToolButton] Track Name %s, Button Name %s", convertToStdString(trackName).c_str(), convertToStdString(btnName).c_str());
	if(trackName != "PIM"){
		UI_WARN("trackName != PIM");
		return;
	}
	if(btnName == "auto_switch") {
		setAutoSwitch(!_autoSwitch, false);
	}
	else if(btnName == "live") {
		pim_station->startStreaming(state ? "camera" : "");
		if(!state)
			return;
		setAutoSwitch(false);
		Q_EMIT setTabViewIndex(0);
	}
	else if(btnName == "export"){
		master_app->dashboard_vm->setListModel(false);
		master_app->dashboard_vm->setExpandDir(2);
	}
}

void PimVM::setAutoSwitch(bool state, bool emit)
{
	if(_autoSwitch = state)
		return;
	_autoSwitch = state;
	if(!emit)
		return;
	Q_EMIT autoSwitchChanged(state);
}

void PimVM::tabViewChange(int index)
{
	if(index < 0)
		return;
	master_app->config.camsFocus.at(3) = cameraFocusToString[static_cast<CAMERAS_FOCUS>(index + 30)];
	master_app->config.writeFileConfig();
}

void PimVM::loadImageFromFile(QString path)
{
	return;
}

void PimVM::tabIndexChange(int index)
{
	return;
}

void PimVM::setAfterUiCreated()
{
	Q_EMIT autoSwitchChanged(_autoSwitch);
	QTimer::singleShot(1 * 1000, this, SLOT(delaySendSignal()));
	// Q_EMIT setStateConnectionTomOImaging(true);
}

void PimVM::delaySendSignal()
{
	Q_EMIT setTabViewIndex(master_app->config.getCameraFocusFromString(master_app->config.camsFocus.at(3)) - 30);
}

void PimVM::clearResult()
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

bool PimVM::changeMxId(std::string mxId)
{
	pim_station->changeMxIdCam(mxId);
}

void PimVM::setSuperUserAct(bool value)
{
	_isSuperUserAct = value;
}
