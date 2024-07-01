#ifndef SYSTEM_LOG_VM_H
#define SYSTEM_LOG_VM_H

#pragma once

#include "../master_app.h"
#include "base_terminal.h"

class SystemLogVM : public QObject
{
	Q_OBJECT
	Q_PROPERTY(bool serverReady READ serverReady WRITE setServerReady NOTIFY serverReadyChanged)
	Q_PROPERTY(bool plannerReady READ plannerReady WRITE setPlannerReady NOTIFY plannerReadyChanged)
	Q_PROPERTY(bool sequenceReady READ sequenceReady WRITE setSequenceReady NOTIFY sequenceReadyChanged)
public:
	explicit SystemLogVM(MasterApp* masterApp, QObject* parent = nullptr);
	~SystemLogVM();

public:
	void createdConn();

	bool getVisible();

private:
	MasterApp* master_app;

	std::string log_dir;
	std::string log_style;
	std::string _currentDay;
	std::string _currentHour;
	std::mutex _getLineThreadMutex;
	std::condition_variable _getLineThreadcv;
	std::vector<std::shared_ptr<std::ofstream>> log_files;
	std::shared_ptr<std::thread> _getLineThread;

	QMap<int, QString> execu_file_name;
	QMap<int, QString> execu_cmd;
	QMap<int, QSharedPointer<BaseTerminal>> execu_terminal;
	QMap<int, QString> termial_title_name;

	int _currentHeight;
	bool _currentVisible;
	bool _isComponentCompleted;
	bool _isPowerUp	  = false;
	bool _isStarted	  = false;
	bool _stateThread = false;
	bool _closeGetLineThread;

	bool _serverReady	= false;
	bool _plannerReady	= false;
	bool _sequenceReady = false;

	void changeThreadState(bool);
	void startTerminalFunc(int);
	void requestHeightToShow();
	void closeGetLineThread();
	void writeAndSend();
	void closeFile();
	int getLineFromQueueThread();
	void logReset();

	QString textWM;
	QString textDM;
	QString textFM;
	QString textCM;
	QString textIM;
	QString textOM;
	QString textTM;
	QString textGM;
	QString textLM;
	QString textSM;

Q_SIGNALS:
	void setContentTerminal(int index, QString logText);
	void setTitleTerminal(QList<QString> list_title);
	void setConfigTerminal(int index, QString bg_color, QString text_font, int text_size, int line_spacing, int limit_line,
						   int scroll_line);
	void setStateButtonTerminal(int index, QString text);
	void sendStateButtonCmd(int index, QString text, bool isButton);
	void resetTerminal(int index);

	void requestHidden();
	void requestShow(int lastHeight);
	void initSystemlog(int initHeight, int initY, int initIndex);

	void setRvizReadyToCall(bool);
	void camsListChanged(QStringList);

	void textServerChanged();
	void sendSequenceReady(bool isSequenceReady);
	void setEnableListModesStates(bool);

	void serverReadyChanged(bool serverReady);
	void plannerReadyChanged(bool plannerReady);
	void sequenceReadyChanged(bool sequenceReady);

public Q_SLOTS:
	void getServerWM(QString);
	void getServerDM(QString);
	void getServerFM(QString);
	void getServerCM(QString);
	void getServerIM(QString);
	void getServerOM(QString);
	void getServerTM(QString);
	void getServerGM(QString);
	void getServerLM(QString);
	void getServerSM(QString);

	void saveServerLog(std::string moduleName);
	void visibleChanged(bool);
	void systemLogClicked(bool);
	void getCurrentHeight(int);
	void getStateTerminal(int, QString);
	void buttonCmdClicked(int, QString, bool);
	void getCurrentTab(int);
	void getCurrentY(int);

	void getLineTerminal(int, QString);
	void setTitleOfTab();
	void getConfigTerminalChanged();
	void callSetConfigTerminal(QString);

	void uiCreatedComplete();
	void getClearTerminal(int);

	bool serverReady();
	bool plannerReady();
	bool sequenceReady();

	void setServerReady(bool);
	void setPlannerReady(bool);
	void setSequenceReady(bool);

	void startControl(bool);
	void getCamsList(QStringList);
	void killAll();
	void sendSignalClosing();
};

#endif	// SYSTEM_LOG_VM_H
