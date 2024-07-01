#pragma once

#include "../master_app.h"
#include "src/AICore/station_core.h"

class PimVM : public QObject
{
	Q_OBJECT
	Q_PROPERTY(int currentTabViewModel MEMBER _currentTabViewModel READ currentTabViewModel)

public:
	explicit PimVM(MasterApp* masterApp, QObject* parent = nullptr);
	~PimVM();

public:
	int currentTabViewModel();

private:
	MasterApp* master_app;
	QHash<QString, QSharedPointer<TrackVM>> track_view_models;
	std::shared_ptr<StationCore> pim_station;

	int getTabViewModel();
	bool changeMxId(std::string mxId);
	bool refreshImage(int docIndex, StreamData& streamData);
	void createConnection();
	void setAutoSwitch(bool value, bool emit = true);

	int _currentTabViewModel;
	bool _autoSwitch = false;
	bool _isSuperUserAct;

Q_SIGNALS:
	void setTabViewIndex(int docIndex);
	void autoSwitchChanged(bool value);
	void setStateConnectionTomOImaging(bool value);

public Q_SLOTS:
	void clearResult();
	void delaySendSignal();
	void tabViewChange(int);
	void setAfterUiCreated();
	void tabIndexChange(int);
	void setSuperUserAct(bool);
	void refreshFrame(int docIndex);
	void loadImageFromFile(QString);
	void clickToolButton(QString, QString, bool);
};
