#ifndef BASE_TERMINAL_H
#define BASE_TERMINAL_H

#pragma once

#include <QMap>
#include <QThread>

#include "../../Script/Config/cfg_app.h"
#include "../master_app.h"
#include <chrono>
#include <errno.h>
#include <fcntl.h>
#include <fstream>
#include <iostream>
#include <ros/ros.h>
#include <signal.h>
#include <sstream>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <string>
#include <sys/wait.h>
#include <thread>
#include <unistd.h>
#include <unordered_map>

#include <boost/algorithm/string.hpp>

#include <queue>
class BaseTerminal : public QThread
{
	Q_OBJECT

public:
	explicit BaseTerminal(MasterApp* masterApp, std::string terminalName, int maxLineCount, QObject* parent = nullptr);
	~BaseTerminal();

	BaseTerminal();

	void run() override;

	void setIndex(int);
	void quitCommand();
	void setCommand(const QString);
	void separateServerSignal(std::string terminalText);

	std::queue<std::string> _lineQueue;
	bool isKilled = false;

private:
	MasterApp* master_app;
	FILE* popen2(std::string command, std::string type, int& pid);
	int pclose2(FILE* fp, pid_t pid);
	pid_t _pid;

	void startCommand();
	void checkNumOfLine(const std::string& line);
	void processColorTags(std::string& terminalLog, std::string& lastColorTag);
	void quitAndWait();
	void resetStatusOfTerminal();
	void callQuitTimer();

	std::queue<std::string> _allContent;
	std::string _terminalName;

	int _maxLineCount;
	int _index;
	int _limitLine;
	int camCounter = 0;
	bool _isQuit;
	int quitTimeOut = 5000;

	QMap<QString, QString> tag_color;
	QString _cmd;
	QStringList camerasID = {"18443010611D631200", "184430108165940F00", "1844301041F6621200"};

Q_SIGNALS:
	void sendLine(int, QString);
	void sendStateCmd(int, QString);
	void sendCamsList(QStringList);
	void clearCmd(int);

	void sendServerWM(QString);
	void sendServerDM(QString);
	void sendServerFM(QString);
	void sendServerCM(QString);
	void sendServerIM(QString);
	void sendServerOM(QString);
	void sendServerTM(QString);
	void sendServerGM(QString);
	void sendServerLM(QString);
	void sendServerSM(QString);

	void sendServerReady(bool);
	void sendPlannerReady(bool);
	void sendSequenceReady(bool);
	void killAllTabDirecly();
	void sendMessageToControl();

public Q_SLOTS:
	void stopCommand(int);
	void startAgainCommand();
	void getStateButtonCommand(int, QString, bool);
	void callPkill();
};

#endif	// BASE_TERMINAL_H
