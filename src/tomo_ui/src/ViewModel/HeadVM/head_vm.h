#ifndef HEADVM_H
#define HEADVM_H
#pragma once

#include "../master_app.h"
#include "src/AICore/station_core.h"

class HeadVM : public QObject
{
	Q_OBJECT
	Q_PROPERTY(int currentTabViewModel MEMBER _currentTabViewModel READ currentTabViewModel)

public:
	explicit HeadVM(MasterApp* masterApp, QObject* parent = nullptr);
	~HeadVM();

public:
	int currentTabViewModel();

private:
	MasterApp* master_app;
	QHash<QString, QSharedPointer<TrackVM>> track_view_models;
	std::shared_ptr<StationCore> tomoeye_station;

	int getTabViewModel();
	bool changeMxId(std::string mxId);
	bool refreshImage(int docIndex, StreamData& streamData);
	void createConnection();
	void setAutoSwitch(bool state, bool emit = true);

	int _currentTabViewModel;
	bool _autoSwitch = false;
	bool _isSuperUserAct;

Q_SIGNALS:
	void setTabViewIndex(int docIndex);
	void autoSwitchChanged(bool value);
	void liveSwitchChanged(bool value);
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
#endif	// HEADVM_H
