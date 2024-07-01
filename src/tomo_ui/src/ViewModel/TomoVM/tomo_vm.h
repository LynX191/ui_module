#ifndef TOMOVM_H
#define TOMOVM_H

#pragma once

#include <sys/stat.h>
#include <sys/types.h>

// My include
#include "../../Script/Define/struct_def.h"
#include "../DashboardVM/dash_board_vm.h"
#include "../ProductionVM/production_vm.h"
#include "../StationVM/station_vm.h"
#include "../master_app.h"
#include "Feature/mode_tomo_vm.h"
#include "Feature/state_tomo_vm.h"
#include "tomo_rviz/visualizer_app.h"

class ProductionVM;

class TomoVM : public QObject
{
	Q_OBJECT
	Q_PROPERTY(QVector<TrackVM*> listTrackVM READ listTrackVM WRITE setListTrackVM NOTIFY listTrackVMChanged)

public:
	explicit TomoVM(int& argc, char** argv, MasterApp* masterApp, QApplication* qapp, QObject* parent = nullptr);
	~TomoVM();

public:
	Q_INVOKABLE QObject* trackListAt(int index) const;

	QHash<QString, QSharedPointer<TrackVM>> track_view_models;
	std::shared_ptr<rviz::VisualizerApp> rviz_app;

	ModeTomoView* modeTomoView;
	StateTomoView* stateTomoView;
	QVector<TrackVM*> listTrackVM();

	void setModeTomo(std::string, int);
	void setStateTomo();
	void closeRvizApp();

	bool isClosing;

protected:
	void parseMessage(const std::string& message, tVectorS& messages, char messagePrefix);
	void parseMessage(const std::string& message, tVectorS& messages, tVectorI& parms);
	void processStatistics(const std::string& stats);
	void processErrorMessage(const std::string& errorMessage);
	void processParameters(const std::string& parmMessage);

	ProductionVM* production_vm;

private:
	QVector<TrackVM*> _listTrackVM;

	MasterApp* master_app;
	QApplication* m_qapp;
	int m_argc;
	char** m_argv;

	tVectorS modes;

	std::shared_ptr<std::thread> comm_thread;
	std::shared_ptr<std::thread> comm_node_thread;
	int commThread();
	int launchCommNode();

	bool parseConfigFile();
	void createConnection();
	void callRvizVisualizer();

	bool toControl{true};
	bool _isRvizFullScreen = false;
	bool _isRvizReadyToCall;
	bool _isRvizCalled;
	bool _isSuperUserAct;

Q_SIGNALS:
	void setProcessState(QString);
	void setLotState(QString);
	void listTrackVMChanged();

	void buttonDocStatusChanged(QString camName, QString docName, bool currentIsClick);
	void setStateConnectionTomOControl(bool value);
	void setTabViewIndex(int docIndex);

	void requestConnectRviz();
	void requestDisconnectRviz();
	void enableAllLayouts(bool);
	void sendRVizMetric(bool, int, int, int, int, bool);
	void sendRvizVisible(bool);
	void initRvizStartState();
	void setEnableRvizBtn(bool value);

public Q_SLOTS:
	void setListTrackVM(QVector<TrackVM*>&);
	int setProductionControl(int, std::string);
	void doubleClickSlots(QString);
	void broadcastPressButtonSlot(QString, QString, bool);
	void clickToolButton(QString, QString, bool);
	void tabViewChange(int);
	void stateIsChanged(std::string);
	void modeIsChanged(std::string);
	void setEnbaleModesStates(bool);
	void testAlarm(int, QString);
	void tabIndexChange(int);
	void initRvizWindow();
	void setAfterUiCreated();
	void callRvizClass();
	void hiddenRvizWindow();
	void updateRVizMetric(bool, int, int, int, int, bool);
	void updateRvizVisible(bool);
	void setRvizReadyToCall(bool);
	void setSuperUserAct(bool);
};

#endif	// TOMOVM_H
