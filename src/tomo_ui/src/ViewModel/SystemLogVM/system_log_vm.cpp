#include "system_log_vm.h"

#include <QDir>
#include <QDirIterator>
#include <QTimer>

#include "../../Script/Config/cfg_app.h"
#include "../../Script/Utilities/utilities.h"

#include <boost/algorithm/string.hpp>
#include <fstream>
#include <iostream>
#include <ros/ros.h>
#include <sys/stat.h>

#include <tomo_devices_carton/def_server.h>

SystemLogVM::SystemLogVM(MasterApp* masterApp, QObject* parent) : master_app{masterApp}, QObject(parent)
{
	std::string execu_path = getenv("HOME") + std::string("/tomo_config/ssh/");

	if(!checkPathExists(execu_path)) {
		UI_WARN("[SystemLogVM::SystemLogVM] Not found folder ssh in %s", execu_path.c_str());
	}
	else {
		QDirIterator it(convertToQString(execu_path), QStringList() << "*.exp", QDir::Files);
		log_dir = std::string(getenv("HOME")) + "/tomo_stats/log/log_" + getCurrentTime(true) + "/";
		boost::replace_all(log_dir, ":", "_");
		mkdir(log_dir.c_str(), 0777);
		log_style = "<body style='font-family:" + master_app->config.textFontCmd +
					"; font-size:" + std::to_string(master_app->config.textSizeCmd) + "px; color:white; background-color:black;'>\n";

		while(it.hasNext()) {
			QString file				   = QFileInfo(it.next()).fileName();
			QString file_without_extension = QFileInfo(file).baseName();
			int index					   = file_without_extension.split("_").at(0).toInt();
			QString name				   = file_without_extension.split("_").at(1);

			execu_file_name[index] = file;
			execu_cmd[index]	   = "expect " + convertToQString(execu_path) + file;
			execu_terminal[index] =
				QSharedPointer<BaseTerminal>::create(master_app, name.toStdString(), master_app->config.limitLineCmd, this->parent());
			termial_title_name[index] = name;
		}
		log_files.resize(execu_file_name.size());
	}

	_currentVisible		  = false;
	_currentHeight		  = 0;
	_isComponentCompleted = false;
	_getLineThread		  = nullptr;
	_getLineThreadcv.notify_all();
	createdConn();
}

SystemLogVM::~SystemLogVM()
{
	closeGetLineThread();
	closeFile();
}
void SystemLogVM::closeFile()
{
	for(auto logFile : log_files) {
		if(logFile->is_open())
			logFile->close();
	}
	log_files.clear();
}
void SystemLogVM::closeGetLineThread()
{
	_closeGetLineThread = true;
	_getLineThreadcv.notify_all();
	if(_getLineThread) {
		_getLineThread->join();
		_getLineThread.reset();
	}
}

void SystemLogVM::createdConn()
{
	Q_FOREACH(QSharedPointer<BaseTerminal> value, execu_terminal.values()) {
		connect(value.data(), SIGNAL(sendLine(int, QString)), this, SLOT(getLineTerminal(int, QString)), Qt::QueuedConnection);
		connect(value.data(), SIGNAL(sendStateCmd(int, QString)), this, SLOT(getStateTerminal(int, QString)), Qt::QueuedConnection);
		connect(this, SIGNAL(sendStateButtonCmd(int, QString, bool)), value.data(), SLOT(getStateButtonCommand(int, QString, bool)),
				Qt::QueuedConnection);
		connect(value.data(), SIGNAL(clearCmd(int)), this, SLOT(getClearTerminal(int)), Qt::QueuedConnection);
		connect(value.data(), SIGNAL(sendPlannerReady(bool)), this, SLOT(setPlannerReady(bool)), Qt::QueuedConnection);
		connect(value.data(), SIGNAL(sendServerReady(bool)), this, SLOT(setServerReady(bool)), Qt::QueuedConnection);
		connect(value.data(), SIGNAL(sendSequenceReady(bool)), this, SLOT(setSequenceReady(bool)), Qt::QueuedConnection);
		connect(value.data(), SIGNAL(sendCamsList(QStringList)), this, SLOT(getCamsList(QStringList)), Qt::QueuedConnection);
		connect(value.data(), SIGNAL(sendServerWM(QString)), this, SLOT(getServerWM(QString)), Qt::QueuedConnection);
		connect(value.data(), SIGNAL(sendServerDM(QString)), this, SLOT(getServerDM(QString)), Qt::QueuedConnection);
		connect(value.data(), SIGNAL(sendServerFM(QString)), this, SLOT(getServerFM(QString)), Qt::QueuedConnection);
		connect(value.data(), SIGNAL(sendServerCM(QString)), this, SLOT(getServerCM(QString)), Qt::QueuedConnection);
		connect(value.data(), SIGNAL(sendServerIM(QString)), this, SLOT(getServerIM(QString)), Qt::QueuedConnection);
		connect(value.data(), SIGNAL(sendServerOM(QString)), this, SLOT(getServerOM(QString)), Qt::QueuedConnection);
		connect(value.data(), SIGNAL(sendServerTM(QString)), this, SLOT(getServerTM(QString)), Qt::QueuedConnection);
		connect(value.data(), SIGNAL(sendServerGM(QString)), this, SLOT(getServerGM(QString)), Qt::QueuedConnection);
		connect(value.data(), SIGNAL(sendServerLM(QString)), this, SLOT(getServerLM(QString)), Qt::QueuedConnection);
		connect(value.data(), SIGNAL(sendServerSM(QString)), this, SLOT(getServerSM(QString)), Qt::QueuedConnection);
		connect(value.data(), SIGNAL(killAllTabDirecly()), this, SLOT(killAll()), Qt::QueuedConnection);
		connect(value.data(), SIGNAL(sendMessageToControl()), this, SLOT(sendSignalClosing()), Qt::QueuedConnection);
	}
}

void SystemLogVM::sendSignalClosing()
{
	UI_INFO("SystenLogVM::sendSignalClosing");
	master_app->control_comm->sendMessage(tomo_peripherals::CMD_UI + std::to_string(tomo_peripherals::CMD_UI_QUIT));
}
void SystemLogVM::startTerminalFunc(int index)
{
	std::shared_ptr<std::ofstream> logFile = std::make_shared<std::ofstream>();

	std::string curTime = getCurrentTime(false, false, false);
	boost::replace_all(curTime, ":", ".");
	std::string logFilePath = log_dir + std::to_string(index) + "_" + curTime + "_" + termial_title_name[index].toStdString() + ".htm";
	logFile->open(logFilePath);
	*logFile << log_style;

	log_files[index - 1] = logFile;
	UI_WARN("SystemLog Terminal %d: Logging process messages to %s", index, logFilePath.c_str());
	if(!_isPowerUp) {
		UI_WARN("[SystemLogVM::startTerminalFunc] Starting terminal at index %d", index);
		execu_terminal.value(index).data()->isKilled = false;
		execu_terminal.value(index).data()->clearCmd(index);
		execu_terminal.value(index).data()->start();
		if(index == execu_file_name.size()) {
			_isStarted = true;
			// _closeGetLineThread = false;
			// if(!_getLineThread)
			// 	_getLineThread = std::make_shared<std::thread>(&SystemLogVM::getLineFromQueueThread, this);
		}
	}
}

void SystemLogVM::getLineTerminal(int index, QString logText)
{
	if(_isStarted)
		logReset();
	if(log_files[index - 1] != nullptr && log_files.size() > index - 1) {
		*log_files[index - 1] << logText.toStdString() << "<br>";
		*log_files[index - 1] << std::flush;
	}
	Q_EMIT setContentTerminal(index, logText);
}

void SystemLogVM::changeThreadState(bool value)
{
	if(_stateThread != value) {
		_stateThread = value;
		_getLineThreadcv.notify_all();
	}
}
int SystemLogVM::getLineFromQueueThread()
{
	while(!_closeGetLineThread) {
		std::unique_lock<std::mutex> lock(_getLineThreadMutex);
		_getLineThreadcv.wait(lock, [this] { return _closeGetLineThread || _stateThread; });

		if(_closeGetLineThread) {
			writeAndSend();
			closeFile();
			return -1;
		}

		if(_isStarted)
			logReset();

		writeAndSend();

		lock.unlock();
	}
	return 0;
}
void SystemLogVM::writeAndSend()
{
	int countEmptyQueue = 0;
	do {
		Q_FOREACH(int key, execu_terminal.keys()) {
			if(execu_terminal.value(key).data()->_lineQueue.empty()) {
				countEmptyQueue++;
				continue;
			}
			std::string data = execu_terminal.value(key).data()->_lineQueue.front();
			if(log_files[key - 1] != nullptr && log_files.size() > key - 1) {
				*log_files[key - 1] << data << "<br>";
				*log_files[key - 1] << std::flush;
				Q_EMIT setContentTerminal(key, convertToQString(data));
				execu_terminal.value(key).data()->_lineQueue.pop();
			}
		}

	} while(countEmptyQueue != execu_terminal.size());
}
void SystemLogVM::logReset()
{
	if(_currentDay != getCurrentDay()) {
		// changeThreadState(false);
		_currentDay = getCurrentDay();
		log_dir		= std::string(getenv("HOME")) + "/tomo_stats/log/log_" + _currentDay + "_00_00_00" + "/";
		boost::replace_all(log_dir, ":", "_");
		mkdir(log_dir.c_str(), 0777);
	}
	if(_currentHour != getCurrentHour()) {
		// changeThreadState(false);
		_currentHour = getCurrentHour();
		_isPowerUp	 = true;
		closeFile();
		log_files.resize(execu_file_name.size());
		startControl(true);
	}
	// changeThreadState(true);
}

bool SystemLogVM::serverReady()
{
	return _serverReady;
}

bool SystemLogVM::plannerReady()
{
	return _plannerReady;
}

bool SystemLogVM::sequenceReady()
{
	return _sequenceReady;
}

void SystemLogVM::setServerReady(bool value)
{
	_serverReady			= value;
	master_app->serverReady = value;
	serverReadyChanged(value);
}

void SystemLogVM::setPlannerReady(bool value)
{
	_plannerReady			 = value;
	master_app->plannerReady = value;
	Q_EMIT setRvizReadyToCall(value);
	plannerReadyChanged(value);
}

void SystemLogVM::setSequenceReady(bool value)
{
	_sequenceReady			  = value;
	master_app->sequenceReady = value;
	if(value) {
		master_app->production_vm->setDryRunValue(master_app->production_vm->_dryRunValue);
		master_app->production_vm->setBypassValue(master_app->production_vm->_bypassValue);
		master_app->production_vm->setSoundState(master_app->production_vm->_soundState);
		master_app->production_vm->setProductId(master_app->production_vm->_productId);
		master_app->station_vm->requestParmList(TOMO_STATION);
		master_app->cfgConfig = master_app->config.configValid;
		if(!master_app->ioMapConfig || !master_app->smConfig || !master_app->cfgConfig || !master_app->rvizConfig) {
			UI_ERROR("Configuration file error");
			master_app->control_comm->sendMessage(tomo_peripherals::CMD_UI + std::to_string(tomo_peripherals::CMD_UI_FAIL));
		}
	}
	sequenceReadyChanged(value);
}

void SystemLogVM::getCurrentY(int y)
{
	if(!_isComponentCompleted) {
		return;
	}
	if(y >= 1080)
		y = 1080;
	if(y <= 0)
		y = 0;
	master_app->config.yOfSystemlog = y;
	master_app->config.writeFileConfig();
}

void SystemLogVM::requestHeightToShow()
{
	Q_EMIT requestShow(_currentHeight);
}

void SystemLogVM::setTitleOfTab()
{
	QList<QString> title = termial_title_name.values();
	Q_EMIT setTitleTerminal(title);
}

void SystemLogVM::callSetConfigTerminal(QString terminal)
{
	int index = termial_title_name.key(terminal);

	Q_EMIT setConfigTerminal(index, convertToQString(master_app->config.bgColorCmd), convertToQString(master_app->config.textFontCmd),
							 master_app->config.textSizeCmd, master_app->config.lineSpacingCmd, master_app->config.limitLineCmd,
							 master_app->config.scrollLineCmd);
}
void SystemLogVM::getConfigTerminalChanged()
{
	for(int index = 1; index <= execu_file_name.size(); index++) {
		Q_EMIT setConfigTerminal(index, convertToQString(master_app->config.bgColorCmd), convertToQString(master_app->config.textFontCmd),
								 master_app->config.textSizeCmd, master_app->config.lineSpacingCmd, master_app->config.limitLineCmd,
								 master_app->config.scrollLineCmd);
	}
}
void SystemLogVM::uiCreatedComplete()
{
	_isComponentCompleted = true;

	master_app->config.yOfSystemlog		 = qRound(1080 * 4 / 5.0) + 1;
	master_app->config.heightOfSystemlog = qRound(1080 * 1 / 5.0) - 1;

	Q_EMIT initSystemlog(master_app->config.heightOfSystemlog, master_app->config.yOfSystemlog, master_app->config.tabOfSystemlog);
}

void SystemLogVM::getClearTerminal(int index)
{
	Q_EMIT resetTerminal(index);
}
void SystemLogVM::visibleChanged(bool visible)
{
	_currentVisible = visible;
}
bool SystemLogVM::getVisible()
{
	return _currentVisible;
}

void SystemLogVM::systemLogClicked(bool visible)
{
	if(visible)
		Q_EMIT requestHidden();
	else
		requestHeightToShow();
}

void SystemLogVM::getCurrentHeight(int height)
{
	if(!_isComponentCompleted) {
		return;
	}
	_currentHeight = height;
	if(_currentHeight >= 1080)
		_currentHeight = 1080;
	if(_currentHeight <= 0)
		_currentHeight = 0;
	master_app->config.heightOfSystemlog = _currentHeight;
	master_app->config.writeFileConfig();
}

void SystemLogVM::getStateTerminal(int index, QString state)
{
	Q_EMIT setStateButtonTerminal(index, state);
}

void SystemLogVM::buttonCmdClicked(int index, QString text, bool isButton)
{
	if(text != "Stop")
		Q_EMIT sendStateButtonCmd(index, text, isButton);
	else
		master_app->modalDialog.getModalDialog(true, "terminal_off_" + std::to_string(index));
}

void SystemLogVM::getCurrentTab(int index)
{
	if(!_isComponentCompleted)
		return;

	master_app->config.tabOfSystemlog = index;
	master_app->config.writeFileConfig();
}

void SystemLogVM::getCamsList(QStringList camsList)
{
	for(int i = 0; i < camsList.size(); i++) {
		master_app->config.camerasID[i] = convertToStdString(camsList[i]);
	}
	master_app->config.writeFileConfig();
	Q_EMIT camsListChanged(camsList);
}
void SystemLogVM::startControl(bool powerValue)
{
	if(powerValue) {
		_currentDay	 = getCurrentDay();
		_currentHour = getCurrentHour();

		Q_FOREACH(int key, execu_terminal.keys()) {
			execu_terminal.value(key).data()->setIndex(key);
			execu_terminal.value(key).data()->setCommand(execu_cmd.value(key));
			QString file_without_extension = QFileInfo(execu_file_name.value(key)).baseName();
			int timeout					   = file_without_extension.split("_").at(2).toInt();
			int index					   = key;
			int timeUnits;
			if(_isPowerUp)
				timeUnits = 0;
			else
				timeUnits = 1000;
			QTimer::singleShot(timeout * timeUnits, [=]() { startTerminalFunc(index); });
		}
	}
	else {
		Q_FOREACH(int key, execu_terminal.keys()) {
			execu_terminal.value(key).data()->setIndex(key);
			execu_terminal.value(key).data()->setCommand(execu_cmd.value(key));
			QString file_without_extension = QFileInfo(execu_file_name.value(key)).baseName();
			int timeout					   = file_without_extension.split("_").at(2).toInt();
			int index					   = key;
			_isPowerUp					   = false;
			if(file_without_extension.indexOf("Sequence") != -1)
				execu_terminal.value(key).data()->getStateButtonCommand(index, "Stop", false);
		}
		_isStarted = false;
	}
}

void SystemLogVM::killAll()
{
	UI_WARN("[SystemLogVM::killAll]");
	for(int i = 0; i <=1; i++){
		Q_FOREACH(int key, execu_terminal.keys()) {
			QString file_without_extension = QFileInfo(execu_file_name.value(key)).baseName();
			switch(i) {
			case 0:
				if(file_without_extension.indexOf("Sequence") != -1)
					execu_terminal.value(key).data()->callPkill();
				break;
			case 1:
				if(file_without_extension.indexOf("Sequence") == -1)
					execu_terminal.value(key).data()->callPkill();
				break;
			}
		}
	}
}
void SystemLogVM::getServerWM(QString serverText)
{
	textWM = textWM + serverText + "<br>";
	saveServerLog("WM");
}

void SystemLogVM::getServerDM(QString serverText)
{
	textDM = textDM + serverText + "<br>";
	saveServerLog("DM");
}

void SystemLogVM::getServerFM(QString serverText)
{
	textFM = textFM + serverText + "<br>";
	saveServerLog("FM");
}

void SystemLogVM::getServerCM(QString serverText)
{
	textCM = textCM + serverText + "<br>";
	saveServerLog("CM");
}

void SystemLogVM::getServerIM(QString serverText)
{
	textIM = textIM + serverText + "<br>";
	saveServerLog("IM");
}

void SystemLogVM::getServerOM(QString serverText)
{
	textOM = textOM + serverText + "<br>";
	saveServerLog("OM");
}

void SystemLogVM::getServerTM(QString serverText)
{
	textTM = textTM + serverText + "<br>";
	saveServerLog("TM");
}

void SystemLogVM::getServerGM(QString serverText)
{
	textGM = textGM + serverText + "<br>";
	saveServerLog("GM");
}

void SystemLogVM::getServerLM(QString serverText)
{
	textLM = textLM + serverText + "<br>";
	saveServerLog("LM");
}

void SystemLogVM::getServerSM(QString serverText)
{
	textSM = textSM + serverText + "<br>";
	saveServerLog("SM");
}

void SystemLogVM::saveServerLog(std::string moduleName)
{
	std::shared_ptr<std::ofstream> logServerFile = std::make_shared<std::ofstream>();
	std::string logModuleFilePatg				 = log_dir + "ServerLog/log_" + moduleName + ".htm";
	logServerFile->open(logModuleFilePatg);
	*logServerFile << log_style;
	if(moduleName == "WM")
		*logServerFile << textWM.toStdString() << "<br>";
	if(moduleName == "DM")
		*logServerFile << textDM.toStdString() << "<br>";
	if(moduleName == "FM")
		*logServerFile << textFM.toStdString() << "<br>";
	if(moduleName == "CM")
		*logServerFile << textCM.toStdString() << "<br>";
	if(moduleName == "IM")
		*logServerFile << textIM.toStdString() << "<br>";
	if(moduleName == "OM")
		*logServerFile << textOM.toStdString() << "<br>";
	if(moduleName == "TM")
		*logServerFile << textTM.toStdString() << "<br>";
	if(moduleName == "GM")
		*logServerFile << textGM.toStdString() << "<br>";
	if(moduleName == "LM")
		*logServerFile << textLM.toStdString() << "<br>";
	if(moduleName == "SM")
		*logServerFile << textSM.toStdString() << "<br>";

	log_files.push_back(logServerFile);
}
