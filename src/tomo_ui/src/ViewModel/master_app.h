#ifndef MASTERAPP_H
#define MASTERAPP_H

#pragma once

#include <memory>

#include <QSharedPointer>

#include "../Dialog/modal_dialogbox.h"
#include "../Script/Config/cfg_app.h"
#include "../Script/Define/struct_def.h"
#include "DashboardVM/dash_board_vm.h"
#include "ProductionVM/production_vm.h"
#include "StationVM/station_vm.h"
#include "track_vm.h"
#include <tomo_comm/topic_string.h>
#include <tomo_devices_carton/client_container.h>
#include <tuple>
#include "../Script/Utilities/utilities.h"

#include <chrono>
#include <ros/ros.h>
#include <sys/stat.h>
#include <sys/statvfs.h>
#include <thread>
#include <dirent.h>
#include <unistd.h>
#include <libssh/libssh.h>
#include <regex>

class StationCore;
class ProductionVM;
class StationVM;
class DashboardVM;

struct Xavier
{
	string ip;
	string name;
	string user;
	string pass;
};

const std::vector<Xavier> XAVIER_DEVICES = {
	// {"192.168.0.221", "xc1", "tomo", "tomo"},
	// {"192.168.0.222", "xc2", "tomo", "tomo"},
	// {"192.168.0.223", "xc3", "tomo", "tomo"},
	// {"localhost","xc4", "tomo", "tomo"}
	{"192.168.0.221", "xc1", "tomo", "tomo"},
	{"192.168.0.221", "xc2", "tomo", "tomo"},
	{"192.168.0.221", "xc3", "tomo", "tomo"},
	{"localhost","xc4", "tomo", "tomo"}
	};

class MasterApp : public QObject
{
	Q_OBJECT
	Q_PROPERTY(bool powerUpState READ powerUpState WRITE setPowerUpState NOTIFY powerUpStateChanged)
	Q_PROPERTY(bool systemReady READ systemReady WRITE setSystemReady NOTIFY systemReadyChanged)
	Q_PROPERTY(bool xavierReady READ xavierReady WRITE setXavierReady NOTIFY xavierReadyChanged)

	Q_PROPERTY(QStringList xavierResources READ xavierResources NOTIFY xavierResourcesChanged)

public:
	explicit MasterApp(bool isSim, QObject* parent = nullptr);
	~MasterApp();
	// property
	bool powerUpState();

public:
	CfgApp config;
	ModalDialogBox modalDialog;

	QScopedPointer<DashboardVM> dashboard_vm;
	QScopedPointer<ProductionVM> production_vm;
	QScopedPointer<StationVM> station_vm;

	QHash<QString, QSharedPointer<TrackVM>> track_view_models;
	QHash<QString, bool> double_click_views;

	static std::shared_ptr<tClientContainer> client;
	std::shared_ptr<TopicString> control_comm;
	std::map<int, std::shared_ptr<StationCore>> stream_tracks;
	std::map<int, bool> track_status;

	bool serverReady   = false;
	bool plannerReady  = false;
	bool sequenceReady = false;

	bool inputCondition = false;

	bool ioMapConfig		= true;
	bool smConfig 			= true;
	bool cfgConfig			= true;
	bool rvizConfig			= true;
	bool checkSuperUser();
	bool isSuperUser  = false;
	void xaviersInfoCheck();
	int checkStorage();
	std::string parent_folder = getenv("HOME") + std::string("/tomo_stats");
	std::string control_folder = getenv("HOME") + std::string("/tomo_stats/log");
	std::string ui_folder = getenv("HOME") + std::string("/tomo_stats/ui_log");
	std::vector<std::string> CheckXavierConnection();
	std::string getDiskStorageInfo(const char* hostname, const char* username, const char* password);
	int terminateReturn(std::string command, int& stateValue , std::string& output);
	void extractNumbers(const std::string& input, int& total, int& used, int& avail, int& usage);

	inline bool isSim()
	{
		return is_sim;
	}


private:
	bool is_sim;

	// property
	bool _powerUpState = false;
	bool _systemReady  = true;
	bool _xavierReady  = true;
	QStringList _xavierResources = {"0-0-0-0", "0-0-0-0", "0-0-0-0", "0-0-0-0"};
	QString wrongTimeXavier;

Q_SIGNALS:
	// property
	void powerUpStateChanged(bool state);
	void setCurrentIndexTab(int);
	void broadcastPressButtonSignal(QString, QString, bool);
	void refreshViewSignal();
	void openAllTerminal(bool isOpen);
	void killTerminal(int, QString, bool);
	void pushBackCurrentPosition(int);
	void confirmExit();

	void systemReadyChanged(bool systemReady);
	void xavierReadyChanged(bool xavierReady);
	void stopProgressBar();
	void xavierResourcesChanged();
	void xavierWrongTime(QString value);
	void sendPowerEnabled(bool value);
	void acceptRestart();
	void readyToUse();
	void parmReset();
	void parmClose();
	void confirmResetExitPass();

public Q_SLOTS:
	// property
	QStringList xavierResources();
	void xaviersTimeCheck();
	void restartXavierNotSync();
	void updateXavierResources(QStringList);
	void setPowerUpState(bool);
	void initStationVM();
	void broadcastPressButton(QString, QString, bool);
	void confirmDialog(QString, bool value);
	void exitDoubleClicked();

	void updateSpeedSlider(int, int);

	void setSystemReady(bool);
	bool systemReady();
	void setXavierReady(bool);
	bool xavierReady();
	bool getDryrunMode();
	bool getBypassMode();
	void setDryrunMode(bool);
	void setBypassMode(bool);
};

#endif	// MASTERAPP_H
