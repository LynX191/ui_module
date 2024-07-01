#pragma once

#include "../master_app.h"
#include "src/AICore/station_core.h"

class ShipperVM : public QObject
{
	Q_OBJECT
	Q_PROPERTY(int currentTabViewModel MEMBER _currentTabViewModel READ currentTabViewModel)

public:
	explicit ShipperVM(MasterApp* masterApp, QObject* parent = nullptr);
	~ShipperVM();

public:
	int currentTabViewModel();

private:
	MasterApp* master_app;
	QHash<QString, QSharedPointer<TrackVM>> track_view_models;
	std::shared_ptr<StationCore> shipper_station;

	int getTabViewModel();
	bool changeMxId(std::string mxId);
	bool refreshImage(int docIndex, StreamData& streamData);
	void createConnection();
	void setAutoSwitch(bool value, bool emit = true);

	int _currentTabViewModel;
	bool _autoSwitch = false;
	bool _isSuperUserAct;

Q_SIGNALS:
	void autoSwitchChanged(bool value);
	void setTabViewIndex(int docIndex);
	void setStateConnectionTomOImaging(bool value);

public Q_SLOTS:

	void clearResult();
	void tabViewChange(int);
	void tabIndexChange(int);
	void setAfterUiCreated();
	void setSuperUserAct(bool);
	void refreshFrame(int docIndex);
	void loadImageFromFile(QString);
	void clickToolButton(QString, QString, bool);
};
