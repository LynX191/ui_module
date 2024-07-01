#include "base_terminal.h"
#include "../../Script/Utilities/utilities.h"
#include <tomo_utils/tomo_utils.h>

#define READ  0
#define WRITE 1

using namespace std;
using namespace std::this_thread;  // sleep_for, sleep_until
BaseTerminal::BaseTerminal(MasterApp* masterApp, string terminalName, int maxLineCount, QObject* parent)
	: master_app{masterApp}, _terminalName(terminalName), _maxLineCount(maxLineCount), QThread(parent)
{
	_cmd	= "";
	_index	= -1;
	_isQuit = false;
	_pid	= 0;
}

BaseTerminal::~BaseTerminal()
{
	_isQuit = true;
	stopCommand(false);
	quitAndWait();
}

BaseTerminal::BaseTerminal()
{
}

void BaseTerminal::quitAndWait()
{
	quit();
	wait();
}
void BaseTerminal::run()
{

	_isQuit = false;
	Q_EMIT sendStateCmd(_index, "Stop");
	startCommand();
}

void BaseTerminal::setIndex(int index)
{
	_index = index;
}

void BaseTerminal::quitCommand()
{
	_isQuit = true;
}

void BaseTerminal::setCommand(const QString cmd)
{
	_cmd = cmd;
}

FILE* BaseTerminal::popen2(string command, string type, int& pid)
{
	pid_t child_pid;
	int fd[2];
	if(pipe(fd) < 0)
		exit(-1);

	if((child_pid = fork()) == -1) {
		perror("fork");
		exit(1);
	}

	/* child process */
	if(child_pid == 0) {
		if(type == "r") {
			close(fd[READ]);	 // Close the READ end of the pipe since the child's fd is write-only
			dup2(fd[WRITE], 1);	 // Redirect stdout to pipe
		}
		else {
			close(fd[WRITE]);	// Close the WRITE end of the pipe since the child's fd is read-only
			dup2(fd[READ], 0);	// Redirect stdin to pipe
		}

		setpgid(child_pid, child_pid);	// Needed so negative PIDs can kill children of /bin/sh
		execl("/bin/sh", "/bin/sh", "-c", command.c_str(), NULL);
		perror("execl");
		exit(1);
	}
	else {
		if(type == "r") {
			close(fd[WRITE]);  // Close the WRITE end of the pipe since parent's fd is read-only
		}
		else {
			close(fd[READ]);  // Close the READ end of the pipe since parent's fd is write-only
		}
	}

	pid = child_pid;

	if(type == "r") {
		return fdopen(fd[READ], "r");
	}

	return fdopen(fd[WRITE], "w");
}

int BaseTerminal::pclose2(FILE* fp, pid_t pid)
{
	int stat = 0;

	fclose(fp);
	while(waitpid(pid, &stat, 0) == -1) {
		if(errno != EINTR) {
			stat = -1;
			break;
		}
	}

	return stat;
}

void BaseTerminal::startCommand()
{
	if(_cmd.isEmpty()) {
		UI_ERROR("[BaseTerminal::startCommand]: Input command is empty! in %d", _index);
		return;
	}

	FILE* script;
	char buffer[1024];
	script = popen2(_cmd.toStdString().c_str(), "r", _pid);
	if(script == NULL) {
		UI_ERROR("[BaseTerminal::startCommand]: Error: Failed to execute command '%s' in %d", _cmd.toStdString().c_str(), _index);
		return;
	}

	UI_WARN("[BaseTerminal #%d]: Started %s", _index, _cmd.toStdString().c_str());
	UI_WARN("PID: %d", _pid);

	string terminalLog, lastColorTag;
	int bufferSize		= sizeof(buffer) - 1;
	bool startLogging	= false;
	bool plannerReady	= false;
	bool serverReady	= false;
	bool sequenceReady	= false;
	bool sequenceClosed = false;
	QString htmlText;
	while(true) {
		memset(&buffer, 0, sizeof(buffer));
		if(read(fileno(script), buffer, bufferSize) == 0) {
			UI_WARN("\n\n[%s] Terminal loop ended: Process stream is empty", _terminalName.c_str());
			break;
		}

		if(_isQuit) {
			UI_WARN("\n\n[%s] Terminal loop ended: Bring down in progress", _terminalName.c_str());
			break;
		}

		terminalLog += buffer;
		int endIndex = terminalLog.rfind("\n");
		if(endIndex != string::npos) {
			try {
				std::string remainingLog = ((endIndex >= terminalLog.size()) ? "" : terminalLog.substr(endIndex + 1));
				terminalLog				 = terminalLog.substr(0, endIndex + 1);
				if(!startLogging && terminalLog.find(" started with pid ") != string::npos)
					startLogging = true;

				if(startLogging) {
					vector<string> splits, subSplits;
					boost::split(splits, terminalLog, boost::is_any_of("\n"));
					for(auto& terminalLog : splits) {
						processColorTags(terminalLog, lastColorTag);
						if(terminalLog.empty())
							continue;

						checkNumOfLine(terminalLog);
						Q_EMIT sendLine(_index, convertToQString(terminalLog));
						// _lineQueue.push(terminalLog);
						if(!plannerReady && terminalLog.find("start planning") != string::npos) {
							plannerReady = true;
							Q_EMIT sendPlannerReady(plannerReady);
							UI_WARN("Planner ready");
						}
						// if(!serverReady && terminalLog.find("carton module servers ready") != string::npos) {
						// 	serverReady = true;
						// 	Q_EMIT sendServerReady(serverReady);
						// 	UI_WARN("Server ready");
						// }
						if(!sequenceReady && terminalLog.find("Waiting for user input") != string::npos) {
							serverReady = true;
							Q_EMIT sendServerReady(serverReady);
							UI_WARN("Server ready");
							
							sequenceReady = true;
							Q_EMIT sendSequenceReady(sequenceReady);
							UI_WARN("Sequence ready");
						}
						if(!sequenceClosed && terminalLog.find("TomO Carton sequence closed") != string::npos) {
							UI_INFO("Kill all terminal directly");
							killAllTabDirecly();
							sequenceClosed = true;
							break;
						}
					}
				}
				terminalLog = remainingLog;
			}
			catch(...) {
				UI_ERROR("Failed to parse system log: %s", terminalLog.c_str());
				terminalLog = "";
			}
		}
	}

	if(_terminalName.find("equence") != string::npos) {
		UI_INFO("[BaseTerminal::startCommand] master_app->control_comm->sendMessage: q");
		master_app->control_comm->sendMessage("q");
		sleep(3);
	}

	// kill(-_pid, 9);
	if(pclose2(script, _pid) == -1)
		// if(kill(-_pid, 9) == -1)
		UI_ERROR("[BaseTerminal #%d]: Error stop %s", _index, _cmd.toStdString().c_str());
	else {
		UI_WARN("[BaseTerminal #%d]: Stopped %s", _index, _cmd.toStdString().c_str());
		Q_EMIT sendStateCmd(_index, "Run");
	}

	resetStatusOfTerminal();

	Q_EMIT sendLine(_index, "\n-------------------- Process ended --------------------\n\n\n\n");
	Q_EMIT sendStateCmd(_index, "Run");
}

void BaseTerminal::resetStatusOfTerminal()
{
	_lineQueue = {};

	if(_terminalName.find("lanner") != string::npos) {
		Q_EMIT sendPlannerReady(false);
		UI_WARN("Planner reset");
	}
	else if(_terminalName.find("erver") != string::npos) {
		Q_EMIT sendServerReady(false);
		UI_WARN("Server reset");
	}
	else if(_terminalName.find("equence") != string::npos) {
		Q_EMIT sendSequenceReady(false);
		UI_WARN("Sequence reset");
	}
}

void BaseTerminal::separateServerSignal(string terminalText)
{
	if(terminalText.find("[WM]") != string::npos | terminalText.find("wagoParam") != string::npos) {
		QString moduleText;
		moduleText = moduleText + convertToQString(terminalText) + "<br>";
		Q_EMIT sendServerWM(moduleText);
	}
	if(terminalText.find("[DM]") != string::npos | terminalText.find("deltaParam") != string::npos) {
		QString moduleText;
		moduleText = moduleText + convertToQString(terminalText) + "<br>";
		Q_EMIT sendServerDM(moduleText);
	}
	if(terminalText.find("[FM]") != string::npos | terminalText.find("foldingParam") != string::npos) {
		QString moduleText;
		moduleText = moduleText + convertToQString(terminalText) + "<br>";
		Q_EMIT sendServerFM(moduleText);
	}
	if(terminalText.find("[CM]") != string::npos | terminalText.find("productInputParam") != string::npos) {
		QString moduleText;
		moduleText = moduleText + convertToQString(terminalText) + "<br>";
		Q_EMIT sendServerCM(moduleText);
	}
	if(terminalText.find("[IM]") != string::npos | terminalText.find("inputPnpParam") != string::npos) {
		QString moduleText;
		moduleText = moduleText + convertToQString(terminalText) + "<br>";
		Q_EMIT sendServerIM(moduleText);
	}
	if(terminalText.find("[OM]") != string::npos | terminalText.find("outputPnpParam") != string::npos) {
		QString moduleText;
		moduleText = moduleText + convertToQString(terminalText) + "<br>";
		Q_EMIT sendServerOM(moduleText);
	}
	if(terminalText.find("[TM]") != string::npos || terminalText.find("trayParam") != string::npos) {
		QString moduleText;
		moduleText = moduleText + convertToQString(terminalText) + "<br>";
		Q_EMIT sendServerTM(moduleText);
	}
	if(terminalText.find("[GM]") != string::npos || terminalText.find("gripperParam") != string::npos) {
		QString moduleText;
		moduleText = moduleText + convertToQString(terminalText) + "<br>";
		Q_EMIT sendServerGM(moduleText);
	}
	if(terminalText.find("[LM]") != string::npos || terminalText.find("loaderParam") != string::npos) {
		QString moduleText;
		moduleText = moduleText + convertToQString(terminalText) + "<br>";
		Q_EMIT sendServerLM(moduleText);
	}
	if(terminalText.find("[SM]") != string::npos || terminalText.find("signalParam") != string::npos) {
		QString moduleText;
		moduleText = moduleText + convertToQString(terminalText) + "<br>";
		Q_EMIT sendServerSM(moduleText);
	}
}

void BaseTerminal::checkNumOfLine(const string& line)
{
	_allContent.push(line);
	if((int) _allContent.size() >= _maxLineCount) {
		for(int i = 0; i < round(_maxLineCount * 0.1); i++)
			_allContent.pop();
	}
}

/// Defines all color mappings from terminal tag to the html color code; Udupa 10Aug'23
map<int, string> colorMap = {{0, "white"},	  {1, "<b>"},	   {30, "black"},	{90, "#808080"}, {31, "#EF391F"}, {91, "#EF190F"},
							 {32, "#39B54A"}, {92, "#31E722"}, {33, "#FFC706"}, {93, "#FFFF00"}, {34, "#3B78FF"}, {94, "#729FCF"},
							 {35, "#D33BD3"}, {95, "#F8B5F8"}, {36, "#33BBC8"}, {96, "#14F0F0"}, {37, "#FFFFFF"}, {97, "#FFFFFF"}};

/// Suppress entire line of log text containing these strings; Udupa 10Aug'23
vector<string> hiddenMessageList  = {};
vector<string> hiddenStartTagList = {};

/// Supress partial texts in log text; Udupa 10Aug'23
// vector<string> hiddenTagList = {"[INFO]", "[WARN]", "[ERROR]"};
vector<string> hiddenTagList = {};

/// New generic implementation for colored text conversion from terminal to html format
/// Parses color coded terminal log text and converts to html tagged format
/// Also hides unnecessary logs
/// Udupa; 10Aug'23
void BaseTerminal::processColorTags(string& terminalLog, string& lastColorTag)
{
	for(auto& hiddenMessage : hiddenMessageList) {
		if(terminalLog.find(hiddenMessage) != string::npos) {
			terminalLog = "";
			return;
		}
	}
	for(auto& hiddenTag : hiddenTagList)
		boost::erase_all(terminalLog, hiddenTag);
	int pos;
	for(auto& hiddenTag : hiddenStartTagList)
		if(!terminalLog.rfind(hiddenTag, 0)) {
			pos = terminalLog.find(']') + 2;
			if(pos < terminalLog.size())
				terminalLog = terminalLog.substr(pos);
			break;
		}
	if(terminalLog.empty())
		return;

	string tag, split, logText;
	int colorCode;
	vector<string> splits, subSplits;

	boost::replace_all(terminalLog, "<", "&lt;");
	boost::replace_all(terminalLog, ">", "&gt;");
	boost::replace_all(terminalLog, "  ", "&nbsp;&nbsp;");

	boost::split(splits, terminalLog, boost::is_any_of("\u001B\0"));
	terminalLog = lastColorTag + splits[0];

	int lastColorCode = lastColorTag.empty() ? -1 : 2;
	for(int i = 1; i < (int) splits.size(); i++) {
		split	= splits[i];
		pos		= split.find('m');
		logText = "";
		if(pos >= 0) {
			tag = split.substr(0, pos);
			boost::split(subSplits, tag, boost::is_any_of("[;"));
			if(subSplits.size()) {
				try {
					colorCode = stoi(subSplits.back());
					if(colorMap.count(colorCode)) {
						tag = colorMap[colorCode];
						if(colorCode == 0) {
							if(lastColorCode == 1)
								logText = "</b>";
							else {
								logText		 = "</span>";
								lastColorTag = "";
							}
						}
						else if(colorCode == 1)
							logText = tag;
						else {
							if(lastColorCode > 1)
								logText = "</span>";
							lastColorTag = "<span style=color:" + colorMap[colorCode] + ">";
							logText += lastColorTag;
						}
						lastColorCode = colorCode;
						if((pos + 1) < split.size())
							logText += split.substr(pos + 1);
					}
				}
				catch(...) {
				}
			}
		}
		else
			logText = split;

		if(logText.empty()) {
			// UI_WARN("Invalid color tag %s", split.c_str());
			logText = split;
		}
		if(!logText.empty()) {
			terminalLog += logText;
		}
	}
	if(lastColorCode > 0)
		terminalLog += lastColorCode == 1 ? "</b>" : "</span>";
}

void BaseTerminal::stopCommand(int tempTimer)
{
	if(_terminalName.find("ision") != string::npos) {
		camCounter = 0;
	}
	// quitCommand();
	if(_pid == 0)
		return;
	UI_WARN("[BaseTerminal::stopCommand]: Kill process with pid %d", _pid);
	if(tempTimer > 0){
		callQuitTimer();
	}
	else
		callPkill();
}

void BaseTerminal::startAgainCommand()
{
	quitAndWait();
	start();
	_allContent = {};
	Q_EMIT clearCmd(_index);
	isKilled = false;
}

void BaseTerminal::getStateButtonCommand(int index, QString text, bool isButton)
{
	if(index != _index)
		return;
	int tempTimer;
	if(isButton){
		tempTimer = 0;
	}
	else{
		tempTimer = quitTimeOut;
	}
	text == "Run" ? startAgainCommand() : stopCommand(tempTimer);
}

void BaseTerminal::callQuitTimer()
{
	std::thread([this]() {
		if(_terminalName.find("equence") != string::npos) {
			sendMessageToControl();
			sleep_for(std::chrono::milliseconds(quitTimeOut));
			if(!isKilled)
				killAllTabDirecly();
		}
	}).detach();
}

void BaseTerminal::callPkill()
{
	UI_WARN("[%s]  CALL PKILL", _terminalName.c_str());
	if(isKilled) return;
	isKilled = true;
	kill(-_pid, 9);
}