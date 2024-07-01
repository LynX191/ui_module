#ifndef SETTING_VM_H
#define SETTING_VM_H
#pragma once

#include <fstream>
#include <ros/ros.h>
#include <yaml-cpp/yaml.h>

// My include
#include "../master_app.h"
// end


using namespace std;
class SettingVM : public QObject
{
	Q_OBJECT
	Q_PROPERTY(QString systemLogFont READ systemLogFont NOTIFY systemLogFontChanged)
	Q_PROPERTY(int systemLogSize READ systemLogSize NOTIFY systemLogSizeChanged)
	Q_PROPERTY(int systemLogSpacing READ systemLogSpacing NOTIFY systemLogSpacingChanged)
	Q_PROPERTY(int systemLogLimit READ systemLogLimit NOTIFY systemLogLimitChanged)
	Q_PROPERTY(int systemLogScroll READ systemLogScroll NOTIFY systemLogScrollChanged)
	Q_PROPERTY(int freeSpace READ freeSpace NOTIFY freeSpaceChanged)

	Q_PROPERTY(int enabledIndex0 READ enabledIndex0 NOTIFY enabledIndex0Changed)
	Q_PROPERTY(int enabledIndex1 READ enabledIndex1 NOTIFY enabledIndex1Changed)
	Q_PROPERTY(int enabledIndex2 READ enabledIndex2 NOTIFY enabledIndex2Changed)
	Q_PROPERTY(int enabledIndex3 READ enabledIndex3 NOTIFY enabledIndex3Changed)
	Q_PROPERTY(int enabledIndex4 READ enabledIndex4 NOTIFY enabledIndex4Changed)
	Q_PROPERTY(int enabledIndex5 READ enabledIndex5 NOTIFY enabledIndex5Changed)
	Q_PROPERTY(int enabledIndex6 READ enabledIndex6 NOTIFY enabledIndex6Changed)

	Q_PROPERTY(QStringList camsListID READ camsListID WRITE setCamsListID NOTIFY camsListIDChanged)
public:
	explicit SettingVM(MasterApp* masterApp, QObject* parent = nullptr);
	~SettingVM();

private:
	MasterApp* master_app;
	QString fileSettingAccount;
	std::string configPath;
	QString driveList;
	QString backupPath;
	std::vector<int> timeVector;
	std::string timeConcat;

	void callBackupScript(std::string);
	void callRemoveScript(std::string, bool isOverWrite);
	void callRestoreScript(std::string);
	QString _systemLogFont = convertToQString(master_app->config.textFontCmd);
	int _systemLogSize	   = master_app->config.textSizeCmd;
	int _systemLogSpacing  = master_app->config.lineSpacingCmd;
	int _systemLogLimit	   = master_app->config.limitLineCmd;
	int _systemLogScroll   = master_app->config.scrollLineCmd;
	int _freeSpace	   = master_app->config.freeSpace;

	int _enabledIndex0 = master_app->config.controlMasterLevel;
	int _enabledIndex1 = master_app->config.productionLevel;
	int _enabledIndex2 = master_app->config.ioControlLevel;
	int _enabledIndex3 = master_app->config.settingLevel;
	int _enabledIndex4 = master_app->config.systemLogLevel;
	int _enabledIndex5 = master_app->config.parametersLevel;
	int _enabledIndex6 = master_app->config.exitPassLevel;
	int _currentAction;
	QStringList _camsListID;

public Q_SLOTS:
	void setAfterUiCreated();

	void saveConfigAs(QString fileName, bool isOverWrite);
	void restoreSavedConfig(QString fileName);
	void resetToDefault();
	void clearAllBackup();
	void deleteFile(const QString fileName);
	void scanLogFilesInFolder();
    void scanLogFilesInFolder2();
	void scanDrive();
	void updateBackupPath(QString);

	void updateCurrentTime(int, int);
	void startSyncTime();

	QString systemLogFont();
	int systemLogSize();
	int systemLogSpacing();
	int systemLogLimit();
	int systemLogScroll();
	int freeSpace();

	void updateFontCmd(QString);
	void updateSizeCmd(int);
	void updateSizeSpacing(int);
	void updateLimitLine(int);
	void updateSystemLogScroll(int);
	void updateFreeSpace(int);

	int enabledIndex0();
	int enabledIndex1();
	int enabledIndex2();
	int enabledIndex3();
	int enabledIndex4();
	int enabledIndex5();
	int enabledIndex6();
	void updatePermissionData(int key, int levelAccess);

	QStringList camsListID();
	void setCamsListID(QStringList);
	void updateCamMxId(QString, QString);
	void resetToDefaultCamID();
	void updateDefaultCamID();
	void setActionList(int action);
	void callReturnAction(QString action, QString folderName = "" ,int transferred = 1, int total = 1);
	void updateImaging(QString, QString, QString);
	void updateControl(QString, QString, QString);
	void updateUi(QString, QString, QString);
	bool checkZipFile(std::string, std::string);
	void openUI();

Q_SIGNALS:
	void systemLogFontChanged();
	void systemLogSizeChanged();
	void systemLogSpacingChanged();
	void systemLogLimitChanged();
	void systemLogScrollChanged();
	void freeSpaceChanged();

	void enabledIndex0Changed();
	void enabledIndex1Changed();
	void enabledIndex2Changed();
	void enabledIndex3Changed();
	void enabledIndex4Changed();
	void enabledIndex5Changed();
	void enabledIndex6Changed();

	void camsListIDChanged();
	void sendFileName(QString name);
	void sendDriveName(QString name);
	void driveListChanged();
	void updateList();
	void enabledImagingBtn();
	void enabledControlBtn();
	void enabledUiBtn();

	void sendUpdateCamToImaging(int, std::string);
	void returnAction(QString action, QString folderName = "" ,int transferred = 1, int total = 1);
};

#endif	// SETTING_VM_H
