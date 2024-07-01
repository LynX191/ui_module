#include "shipper_vm.h"
#include <ros/ros.h>

ShipperVM::ShipperVM(MasterApp* masterApp, QObject* parent) : master_app(masterApp), QObject{parent}
{
	_isSuperUserAct = false;
	if(master_app) {
		shipper_station =
			std::make_shared<StationCore>(STATION_1, "shipper", tVectorS({"Stream", "StimCartoner|0", "StimCartoner|1", "StimCartoner|2"}));
		master_app->stream_tracks[STATION_1] = shipper_station;
		master_app->track_status[STATION_1]	 = shipper_station->initialize();
		track_view_models.insert("Shipper", master_app->track_view_models["Shipper"]);
	}

	createConnection();
	setAutoSwitch(true);
}

ShipperVM::~ShipperVM()
{
}

int ShipperVM::currentTabViewModel()
{
	return _currentTabViewModel;
}

void ShipperVM::createConnection()
{
	connect(shipper_station.get(), SIGNAL(updateImage_Station_VM_Signal(int)), this, SLOT(refreshFrame(int)));
}

int ShipperVM::getTabViewModel()
{
	return currentTabViewModel();
}

bool ShipperVM::refreshImage(int docIndex, StreamData& streamData)
{
	if(!master_app->isSuperUser)
		return false;
	UI_INFO("[HeadVM::refreshImage] Displaying image doc #%d", docIndex);
	if(!streamData.q_image.isNull())
		track_view_models.value("Shipper")->UpdateImage(streamData.q_image, docIndex);
	return true;
}

void ShipperVM::refreshFrame(int docIndex)
{
	if(!master_app->isSuperUser)
		return;
	UI_INFO("[ShipperVM::refreshFrame] View %d", docIndex);
	if(docIndex < 0)
		return;
	StreamData streamData;
	if(!shipper_station->getFrame(docIndex, streamData) || streamData.image_frame.empty()) {
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

void ShipperVM::clickToolButton(QString trackName, QString btnName, bool state)
{
	UI_INFO("[ShipperVM::clickToolButton] Track Name %s, Button Name %s", convertToStdString(trackName).c_str(), convertToStdString(btnName).c_str());
	if(!master_app->isSuperUser)
		return;
	if(trackName != "Shipper")
		return;
	if(btnName == "auto_switch")
		setAutoSwitch(!_autoSwitch, false);
	else if(btnName == "live") {
		if(state) {
			setAutoSwitch(false);
			Q_EMIT setTabViewIndex(0);
		}
		shipper_station->startStreaming(state ? "camera" : "");
	}
	if(btnName == "export"){
		master_app->dashboard_vm->setListModel(false);
		master_app->dashboard_vm->setExpandDir(1);
	}
}

void ShipperVM::setAutoSwitch(bool state, bool emit)
{
	if(_autoSwitch = state)
		return;
	_autoSwitch = state;
	if(!emit)
		return;
	Q_EMIT autoSwitchChanged(state);
}

void ShipperVM::tabViewChange(int index)
{
	if(index < 0)
		return;
	master_app->config.camsFocus.at(2) = cameraFocusToString[static_cast<CAMERAS_FOCUS>(index + 20)];
	master_app->config.writeFileConfig();
}

void ShipperVM::loadImageFromFile(QString path)
{
	return;
}

void ShipperVM::tabIndexChange(int index)
{
	return;
}

void ShipperVM::setAfterUiCreated()
{
	Q_EMIT autoSwitchChanged(_autoSwitch);
	Q_EMIT setTabViewIndex(master_app->config.getCameraFocusFromString(master_app->config.camsFocus.at(2)) - 20);
	// Q_EMIT setStateConnectionTomOImaging(true);
}

void ShipperVM::clearResult()
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

bool ShipperVM::changeMxId(std::string mxId)
{
	shipper_station->changeMxIdCam(mxId);
}

void ShipperVM::setSuperUserAct(bool value)
{
	_isSuperUserAct = value;
}
