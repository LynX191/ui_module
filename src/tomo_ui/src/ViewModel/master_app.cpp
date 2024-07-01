#include "master_app.h"

std::shared_ptr<tClientContainer> MasterApp::client = nullptr;
using namespace tomo_peripherals;
using namespace std::this_thread;  // sleep_for, sleep_until
using namespace std::chrono_literals;

MasterApp::MasterApp(bool isSim, QObject* parent) : is_sim(isSim), QObject{parent}
{
	config.readFileConfig();

	client = std::make_shared<tClientContainer>(tomo_peripherals::ModuleToRunModes);
	client->createClients(tModuleIds({WAGO_MODULE, DELTA_MODULE}));
	control_comm = nullptr;

	for(int i = 0; i < config.trackNames.size(); i++) {
		track_view_models.insert(config.trackNames.at(i), QSharedPointer<TrackVM>::create(config.docNames.value(config.trackNames.at(i))));
		double_click_views.insert(config.trackNames.at(i), false);
	}

	dashboard_vm.reset(new DashboardVM(parent, this));
	dashboard_vm.data()->setTrackviewModels(track_view_models);

	production_vm.reset(new ProductionVM(parent, this));
	production_vm->readFileStatistics();
	production_vm->setDryRunValue(config.dryrunMode);
	production_vm->setBypassValue(config.bypassMode);
	production_vm->setSoundState(config.soundMode);
	production_vm->setProductId(config.productId);

	station_vm.reset(new StationVM(parent, this));
	initStationVM();
}

MasterApp::~MasterApp()
{
	config.writeFileConfig();
	Q_FOREACH(QSharedPointer<TrackVM> value, track_view_models) {
		delete value.data();
	}
}

bool MasterApp::powerUpState()
{
	return _powerUpState;
}

void MasterApp::setPowerUpState(bool value)
{
	_powerUpState = value;
	Q_EMIT powerUpStateChanged(value);
	if(!value)
		return;
	production_vm->setEndLotEnabled(false);
	production_vm->setStartLotEnabled(true);
	modalDialog.getAlarmConfig();
	// xaviersTimeCheck();
	xaviersInfoCheck();
}

bool MasterApp::checkSuperUser()
{
	return isSuperUser;
}
void MasterApp::xaviersInfoCheck()
{
	if(is_sim)
		return;

	UI_WARN("Storage state: %d", checkStorage());

	try {
		std::thread([this]() {
			vector<string> disconnected_xaviers = CheckXavierConnection();

			if(disconnected_xaviers.empty()) {
				UI_INFO("All Xaviers are connected!");
				setXavierReady(true);
				modalDialog.showErrorMessage(0, "xc1");
				modalDialog.showErrorMessage(0, "xc2");
				modalDialog.showErrorMessage(0, "xc3");
			}
			else {
				UI_ERROR("Disconnected Xaviers");
				for(const auto& xavier : disconnected_xaviers) {
					UI_ERROR("%s", xavier.c_str());
					modalDialog.showErrorMessage(-3, xavier);
				}
				stopProgressBar();
				setXavierReady(false);
			}
		}).detach();
	}
	catch(const exception& e) {
		UI_ERROR("Exception occurred: %s", e);
	}
}

void MasterApp::xaviersTimeCheck()
{
	if(is_sim)
		return;

	try {
		std::thread([this]() {
			Q_EMIT modalDialog.getLoadingPopup("Checking time\nsynchronization");
			QString wrongVector = "000";
			bool timeDiffExist	= false;
			for(int i = 0; i < 3; i++) {
				std::string getTimeCmd;
				std::string user_host = XAVIER_DEVICES[i].user + "@" + XAVIER_DEVICES[i].ip;
				std::string password  = "'" + XAVIER_DEVICES[i].pass + "'";
				getTimeCmd =
					"sshpass -p " + password + " ssh " + user_host + " -o ConnectTimeout=1 -o StrictHostKeyChecking=no  'date +%s'";
				// getTimeCmd = sshpass -p 'tomo' ssh tomo@192.168.0.102 'date "+%H:%M:%S   %d/%m/%y"'
				bool checkTimeDiff = false;
				for(int k = 1; k <= 3; k++){
					int timeXavierReturn, timeXC4Return;
					std::string timeReturnText, currentXC4Time;
					terminateReturn(getTimeCmd, timeXavierReturn, timeReturnText);
					int timeXavierConverted = 0, timeXC4Converted = 0;
					if(timeXavierReturn == 0)
						timeXavierConverted = std::stoi(timeReturnText);
					UI_WARN("Cmd terminate: \n %s", getTimeCmd.c_str());
					UI_WARN("Current time on %s: %s", XAVIER_DEVICES[i].name.c_str(), timeReturnText.c_str());
					std::string getCurrentTimeCmd = "date +%s";
					terminateReturn(getCurrentTimeCmd, timeXC4Return, currentXC4Time);
					UI_WARN("Current time on xc4: %s", currentXC4Time.c_str());
					if(timeXC4Return == 0)
						timeXC4Converted = std::stoi(currentXC4Time);
					int difference	   = abs(timeXC4Converted - timeXavierConverted);
					checkTimeDiff = difference <= 5 && timeXC4Converted != 0;
					if(checkTimeDiff) {
						UI_WARN("Time synced: %s == xc4", XAVIER_DEVICES[i].name.c_str());
						break;
					}
					if(k != 3)
						sleep_for(5000ms);
				}
				if(!checkTimeDiff) {
					stopProgressBar();
					sendPowerEnabled(false);
					timeDiffExist = true;
					UI_ERROR("Time asynchronous: %s != xc4", XAVIER_DEVICES[i].name.c_str());
					QChar replacement = '1';  // Use single quotes for a single character
					wrongVector.replace(i, 1, replacement);
				}
			}
			if(timeDiffExist){
				modalDialog.popupModalDialog(false, -3, "time_fail", convertToQString(ConditionWarning["time_fail"]));
			}
			else{
				sendPowerEnabled(true);
				readyToUse();
				// modalDialog.popupModalDialog(false, -1, "time_success", convertToQString(ConditionWarning["time_success"]));
			}
			Q_EMIT xavierWrongTime(wrongVector);
			wrongTimeXavier = wrongVector;
			Q_EMIT modalDialog.getClosePopupLoading();
		}).detach();
	}
	catch(const exception& e) {
		UI_ERROR("Exception occurred: %s", e);
	}
}

void MasterApp::restartXavierNotSync()
{
	try {
		std::thread([this]() {
			for(int i = 0; i < 3; i++) {
				if(wrongTimeXavier[i] == "0")
					continue;
				std::string user_host = XAVIER_DEVICES[i].user + "@" + XAVIER_DEVICES[i].ip;
				std::string password  = "'" + XAVIER_DEVICES[i].pass + "'";
				int rebootReturn;
				std::string rebootReturnText, rebootCmd;
				rebootCmd =
					"sshpass -p " + password + " ssh " + user_host + " -o ConnectTimeout=5 -o StrictHostKeyChecking=no  sudo reboot";
				terminateReturn(rebootCmd, rebootReturn, rebootReturnText);
				UI_WARN("Cmd terminate: \n %s", rebootCmd.c_str());
			}
		}).detach();
	}
	catch(const exception& e) {
		UI_ERROR("[MasterApp::restartXavierNotSync] Exception occurred: %s", e);
	}
}
std::vector<std::string> MasterApp::CheckXavierConnection()
{
	vector<string> disconnected_xaviers;
	QStringList temp_xaviers;
	// Ping each Xavier and check for disconnection
	for(const auto& xavier : XAVIER_DEVICES) {
		string ping_command = "ping -c 1 -W 1 " + xavier.ip;
		try {
			int ping_result = system(ping_command.c_str());
			if(ping_result != 0) {
				disconnected_xaviers.push_back(xavier.name);
				temp_xaviers.append("0-0-0-0");
			}
			else {
				std::string input = getDiskStorageInfo(xavier.ip.c_str(), xavier.user.c_str(), xavier.pass.c_str());
				if(input != "Failed") {
					UI_WARN("[MasterApp::getDiskStorageInfo]: Result\n%s", input.c_str());
					int total, used, avail, usage;
					extractNumbers(input, total, used, avail, usage);
					UI_WARN("Storage %s \nTotal	Used	Available	Use%%\n%dG	%dG	%dG		%d%%", xavier.name.c_str(), total, used, avail,
							usage);
					QString data =
						QString::number(total) + "-" + QString::number(used) + "-" + QString::number(avail) + "-" + QString::number(usage);
					UI_WARN("Storage: %s", convertToStdString(data).c_str());
					temp_xaviers.append(data);
				}
				else {
					temp_xaviers.append("0-0-0-0");
				}
			}
		}
		catch(const exception& e) {
			UI_ERROR("Exception occurred while pinging Xavier %s: %s", xavier.name.c_str(), e);
		}
	}
	updateXavierResources(temp_xaviers);

	return disconnected_xaviers;
}

QStringList MasterApp::xavierResources()
{
	return _xavierResources;
}

int MasterApp::checkStorage()
{
	// Calculate space values in gigabytes

	std::thread([this]() {
		std::string pathString = parent_folder + "/";
		const char* path	   = pathString.c_str();  // You can specify the path to any directory you're interested in
		struct statvfs fileStat;
		if(statvfs(path, &fileStat) == 0) {
			double totalSpaceGB = static_cast<double>(fileStat.f_bsize * fileStat.f_blocks) / (1000 * 1000 * 1000);
			double freeSpaceGB	= static_cast<double>(fileStat.f_bsize * fileStat.f_bavail) / (1000 * 1000 * 1000);
			// Print space information with UI_WARN
			UI_WARN("Total Space: %.2f GB", totalSpaceGB);
			UI_WARN("Free Space: %.2f GB", freeSpaceGB);
			UI_WARN("Require Space: %d GB", config.freeSpace);
			while(freeSpaceGB < config.freeSpace) {
				// Store subfolder names and their last modification time
				std::vector<std::pair<std::string, time_t>> subfolders;
				std::string currentFolder;
				for(int i = 0; i < 2; i++) {
					std::string currentFolder;
					if(i == 0)
						currentFolder = control_folder;
					else if(i == 1)
						currentFolder = ui_folder;

					// Open the parent directory
					DIR* dir = opendir(currentFolder.c_str());
					if(!dir) {
						UI_ERROR("Error opening parent folder.");
						return 1;
					}

					// Read subfolders from the parent directory
					struct dirent* entry;
					while((entry = readdir(dir)) != nullptr) {
						std::string entryName = entry->d_name;
						if(entryName != "." && entryName != "..") {
							std::string entryPath = currentFolder + "/" + entryName;
							struct stat statbuf;
							if(stat(entryPath.c_str(), &statbuf) == 0) {
								subfolders.emplace_back(entryPath, statbuf.st_mtime);
							}
						}
					}
					closedir(dir);
				}

				// Sort subfolders based on last modification time (oldest first)
				std::sort(subfolders.begin(), subfolders.end(), [](const auto& a, const auto& b) { return a.second < b.second; });
				// Remove the oldest subfolder
				if(!subfolders.empty()) {
					const std::string& subfolder_name = subfolders.front().first;
					const time_t& subfolder_time	  = subfolders.front().second;
					std::string subfolder_path		  = subfolder_name;
					std::string removeCmd			  = "rm -r " + subfolder_path;
					if(system(removeCmd.c_str()) == 0) {
						UI_WARN("Removed: %s, last modified: %ld", subfolder_path.c_str(), subfolder_time);

						// Re-query file system information to get updated free space
						if(statvfs(path, &fileStat) == 0) {
							freeSpaceGB = static_cast<double>(fileStat.f_bsize * fileStat.f_bavail) / (1000 * 1000 * 1000);
							UI_WARN("Free Space After Remove: %.2f GB", freeSpaceGB);
						}
						else {
							UI_ERROR("Error getting file system information after removal.");
							return 1;
						}
					}
					else {
						UI_ERROR("Error removing folder : %s", subfolder_path.c_str());
						return 1;
					}
				}
				else {
					UI_ERROR("No subfolders found.");
					return 1;
				}
			}
		}
		else {
			UI_ERROR("Error getting file system information.");
			return 1;
		}
	}).detach();
	return 0;
}

std::string MasterApp::getDiskStorageInfo(const char* hostname, const char* username, const char* password)
{
	ssh_session sshSession = ssh_new();
	if(sshSession == nullptr) {
		UI_ERROR("Failed to create SSH session.");
		return "Failed";
	}

	ssh_options_set(sshSession, SSH_OPTIONS_HOST, hostname);
	ssh_options_set(sshSession, SSH_OPTIONS_USER, username);

	int rc = ssh_connect(sshSession);
	if(rc != SSH_OK) {
		UI_ERROR("Failed to connect to remote host.");
		return "Failed";
	}

	rc = ssh_userauth_password(sshSession, nullptr, password);
	if(rc != SSH_AUTH_SUCCESS) {
		ssh_disconnect(sshSession);
		ssh_free(sshSession);
		UI_ERROR("Authentication failed.");
		return "Failed";
	}

	ssh_channel channel = ssh_channel_new(sshSession);
	if(channel == nullptr) {
		ssh_disconnect(sshSession);
		ssh_free(sshSession);
		UI_ERROR("Failed to create SSH channel.");
		return "Failed";
	}

	rc = ssh_channel_open_session(channel);
	if(rc != SSH_OK) {
		ssh_channel_free(channel);
		ssh_disconnect(sshSession);
		ssh_free(sshSession);
		UI_ERROR("Failed to open SSH channel session.");
		return "Failed";
	}
	rc = ssh_channel_request_exec(channel, "df -H /");
	if(rc != SSH_OK) {
		ssh_channel_close(channel);
		ssh_channel_free(channel);
		ssh_disconnect(sshSession);
		ssh_free(sshSession);
		;
		UI_ERROR("Failed to execute command.");
		return "Failed";
	}

	std::string output;
	char buffer[256];
	int nbytes;
	while((nbytes = ssh_channel_read(channel, buffer, sizeof(buffer), 0)) > 0) {
		output.append(buffer, nbytes);
	}

	ssh_channel_send_eof(channel);
	ssh_channel_close(channel);
	ssh_channel_free(channel);
	ssh_disconnect(sshSession);
	ssh_free(sshSession);

	return output;
}

int MasterApp::terminateReturn(std::string command, int& stateValue, std::string& output)
{
	FILE* pipe = popen(command.c_str(), "r");
	if(!pipe) {
		std::cerr << "popen failed!" << std::endl;
		stateValue = EXIT_FAILURE;
		return stateValue;
	}

	// Read the command's output into a string
	char buffer[128];
	while(!feof(pipe)) {
		if(fgets(buffer, 128, pipe) != nullptr) {
			output += buffer;
		}
	}
	stateValue = pclose(pipe);
	return stateValue;
}

void MasterApp::extractNumbers(const std::string& input, int& total, int& used, int& avail, int& usage)
{
	// Regular expressions to match numbers followed by "G" and "%" without capturing the "%"
	std::regex regexG("\\b(\\d+)G\\b");
	std::regex regexPercent("\\b(\\d+)%?\\b");

	// Match total, used, and available numbers
	std::smatch match;
	std::string::const_iterator searchStart(input.cbegin());
	for(int i = 0; i < 3; ++i) {
		if(std::regex_search(searchStart, input.cend(), match, regexG)) {
			switch(i) {
			case 0:
				total = std::stoi(match[1]);
				break;
			case 1:
				used = std::stoi(match[1]);
				break;
			case 2:
				avail = std::stoi(match[1]);
				break;
			}
			searchStart = match.suffix().first;
		}
		else {
			// Handle error: Unable to find required number
			return;
		}
	}

	// Match percentage
	if(std::regex_search(searchStart, input.cend(), match, regexPercent)) {
		usage = std::stoi(match[1]);
	}
	else {
		// Handle error: Unable to find percentage
		return;
	}
}

void MasterApp::updateXavierResources(QStringList value)
{
	_xavierResources = value;
	xavierResourcesChanged();
}

bool MasterApp::systemReady()
{
	return _systemReady;
}

void MasterApp::setSystemReady(bool value)
{
	_systemReady = value;
	Q_EMIT systemReadyChanged(value);
}

bool MasterApp::xavierReady()
{
	return _xavierReady;
}

void MasterApp::setXavierReady(bool value)
{
	_xavierReady = value;
	Q_EMIT xavierReadyChanged(value);
}

bool MasterApp::getDryrunMode()
{
	return config.dryrunMode;
}

bool MasterApp::getBypassMode()
{
	return config.bypassMode;
}

void MasterApp::setDryrunMode(bool value)
{
	config.dryrunMode = value;
	config.writeFileConfig();
}

void MasterApp::setBypassMode(bool value)
{
	config.bypassMode = value;
	config.writeFileConfig();
}

void MasterApp::broadcastPressButton(QString camCame, QString docName, bool isClick)
{
	Q_EMIT broadcastPressButtonSignal(camCame, docName, isClick);
}
void MasterApp::initStationVM()
{
	station_vm.data()->setBsSpeedSlider(config.sliderS1);
	station_vm.data()->setTsSpeedSlider(config.sliderS2);
	station_vm.data()->setItSpeedSliderX(config.sliderS3x);
	station_vm.data()->setItSpeedSliderY(config.sliderS3y);
	station_vm.data()->setAfSpeedSliderX(config.sliderS4x);
	station_vm.data()->setAfSpeedSliderY(config.sliderS4y);
	station_vm.data()->setOtSpeedSliderX(config.sliderS9x);
	station_vm.data()->setOtSpeedSliderY(config.sliderS9y);
	station_vm.data()->setOtSpeedSliderZ(config.sliderS9z);
}

void MasterApp::confirmDialog(QString messageID, bool value)
{
	UI_INFO("[MasterApp::confirm] %s : %s", convertToStdString(messageID).c_str(), value ? "Confirm":"Ignore");
	std::thread([this, messageID, value]() {
		if(value){
			if(messageID == "exit_click") {
				if(powerUpState()) {
					setPowerUpState(false);
					while(true) {
						if(!serverReady) {
							sleep_for(5250ms);
							Q_EMIT confirmExit();
							break;
						}
						sleep_for(1000ms);
					}
				}
				else {
					Q_EMIT confirmExit();
				}
			}
			else if(messageID == "bs_home")
				station_vm->homeAxis(BSTM_AXIS);
			else if(messageID == "bs_move")
				station_vm->moveAxis(BSTM_AXIS, station_vm->_bsPosition, 1, station_vm->_bsSpeedSlider);
			else if(messageID == "bs_back")
				station_vm->moveAxis(BSTM_AXIS, station_vm->_bsDistance * -1, 0, station_vm->_bsSpeedSlider);
			else if(messageID == "bs_forw")
				station_vm->moveAxis(BSTM_AXIS, station_vm->_bsDistance, 0, station_vm->_bsSpeedSlider);
			else if(messageID == "ts_home")
				station_vm->homeAxis(TSTM_AXIS);
			else if(messageID == "ts_move")
				station_vm->moveAxis(TSTM_AXIS, station_vm->_tsPosition, 1, station_vm->_tsSpeedSlider);
			else if(messageID == "ts_back")
				station_vm->moveAxis(TSTM_AXIS, station_vm->_tsDistance * -1, 0, station_vm->_tsSpeedSlider);
			else if(messageID == "ts_forw")
				station_vm->moveAxis(TSTM_AXIS, station_vm->_tsDistance, 0, station_vm->_tsSpeedSlider);
			else if(messageID == "it_home") {
				if(!inputCondition) {
					modalDialog.getModalDialog(false, "it_fail");
					return;
				}
				station_vm->homeAxis(station_vm->_itCurrentAxis);
			}
			else if(messageID == "it_home_all") {
				if(!inputCondition) {
					modalDialog.getModalDialog(false, "it_fail");
					return;
				}
				station_vm->homeAxis(INPUT_PNP_X_AXIS);
				station_vm->homeAxis(INPUT_PNP_Y_AXIS);
			}
			else if(messageID == "it_move") {
				if(!inputCondition) {
					modalDialog.getModalDialog(false, "it_fail");
					return;
				}
				switch(station_vm->_itCurrentAxis) {
				case INPUT_PNP_X_AXIS:
					station_vm->moveAxis(station_vm->_itCurrentAxis, station_vm->_itPosition, 1, station_vm->_itSpeedSliderX);
					break;
				case INPUT_PNP_Y_AXIS:
					station_vm->moveAxis(station_vm->_itCurrentAxis, station_vm->_itPosition, 1, station_vm->_itSpeedSliderY);
					break;
				default:
					break;
				}
			}
			else if(messageID == "it_back") {
				if(!inputCondition) {
					modalDialog.getModalDialog(false, "it_fail");
					return;
				}
				switch(station_vm->_itCurrentAxis) {
				case INPUT_PNP_X_AXIS:
					station_vm->moveAxis(station_vm->_itCurrentAxis, station_vm->_itDistance * -1, 0, station_vm->_itSpeedSliderX);
					break;
				case INPUT_PNP_Y_AXIS:
					station_vm->moveAxis(station_vm->_itCurrentAxis, station_vm->_itDistance * -1, 0, station_vm->_itSpeedSliderY);
					break;
				default:
					break;
				}
			}
			else if(messageID == "it_forw") {
				if(!inputCondition) {
					modalDialog.getModalDialog(false, "it_fail");
					return;
				}
				switch(station_vm->_itCurrentAxis) {
				case INPUT_PNP_X_AXIS:
					station_vm->moveAxis(station_vm->_itCurrentAxis, station_vm->_itDistance, 0, station_vm->_itSpeedSliderX);
					break;
				case INPUT_PNP_Y_AXIS:
					station_vm->moveAxis(station_vm->_itCurrentAxis, station_vm->_itDistance, 0, station_vm->_itSpeedSliderY);
					break;
				default:
					break;
				}
			}
			else if(messageID == "ot_home")
				station_vm->homeAxis(station_vm->_otCurrentAxis);
			else if(messageID == "ot_home_all") {
				station_vm->waitHomeAllDone = false;
				station_vm->setReadyToMove(OUTPUT_PNP_X_AXIS, 1, true);
				station_vm->setReadyToMove(OUTPUT_PNP_Y_AXIS, 1, true);
				station_vm->setReadyToMove(OUTPUT_PNP_Z_AXIS, 1, true);

				station_vm->homeAxis(OUTPUT_PNP_Z_AXIS);
				while(station_vm->_otZAxisPosition > 5) {
					if(station_vm->servo_error)
						return;
					sleep_for(1000ms);
				}
				if(station_vm->servo_error)
					return;
				sleep_for(3000ms);
				station_vm->homeAxis(OUTPUT_PNP_X_AXIS);
				station_vm->waitForState = false;
				if(station_vm->servo_error)
					return;
				sleep_for(1000ms);

				station_vm->homeAxis(OUTPUT_PNP_Y_AXIS);
				
				sleep_for(1000ms);
				station_vm->waitHomeAllDone = true;
			}
			else if(messageID == "ot_move")
				switch(station_vm->_otCurrentAxis) {
				case OUTPUT_PNP_X_AXIS:
					station_vm->moveAxis(station_vm->_otCurrentAxis, station_vm->_otPosition, 1, station_vm->_otSpeedSliderX);
					break;
				case OUTPUT_PNP_Y_AXIS:
					station_vm->moveAxis(station_vm->_otCurrentAxis, station_vm->_otPosition, 1, station_vm->_otSpeedSliderY);
					break;
				case OUTPUT_PNP_Z_AXIS:
					station_vm->moveAxis(station_vm->_otCurrentAxis, station_vm->_otPosition, 1, station_vm->_otSpeedSliderZ);
					break;
				default:
					break;
				}
			else if(messageID == "ot_back")
				switch(station_vm->_otCurrentAxis) {
				case OUTPUT_PNP_X_AXIS:
					station_vm->moveAxis(station_vm->_otCurrentAxis, station_vm->_otDistance * -1, 0, station_vm->_otSpeedSliderX);
					break;
				case OUTPUT_PNP_Y_AXIS:
					station_vm->moveAxis(station_vm->_otCurrentAxis, station_vm->_otDistance * -1, 0, station_vm->_otSpeedSliderY);
					break;
				case OUTPUT_PNP_Z_AXIS:
					station_vm->moveAxis(station_vm->_otCurrentAxis, station_vm->_otDistance * -1, 0, station_vm->_otSpeedSliderZ);
					break;
				default:
					break;
				}
			else if(messageID == "ot_forw")
				switch(station_vm->_otCurrentAxis) {
				case OUTPUT_PNP_X_AXIS:
					station_vm->moveAxis(station_vm->_otCurrentAxis, station_vm->_otDistance, 0, station_vm->_otSpeedSliderX);
					break;
				case OUTPUT_PNP_Y_AXIS:
					station_vm->moveAxis(station_vm->_otCurrentAxis, station_vm->_otDistance, 0, station_vm->_otSpeedSliderY);
					break;
				case OUTPUT_PNP_Z_AXIS:
					station_vm->moveAxis(station_vm->_otCurrentAxis, station_vm->_otDistance, 0, station_vm->_otSpeedSliderZ);
					break;
				default:
					break;
				}
			else if(messageID == "terminal_off_all") {
				UI_INFO("<<<<<  Bring down system  >>>>>");
				modalDialog.setIsCovered(true);
				modalDialog.checkModulesState();
				for(const auto& errorCode : modalDialog.ErrorIDToString)
					production_vm->emergencyEvent(false, convertToQString(errorCode.first));
				setPowerUpState(false);
			}
			else if(messageID.indexOf("terminal_off_") != -1) {
				int index	 = messageID.mid(13, 1).toInt();
				QString stop = "Stop";
				Q_EMIT killTerminal(index, stop, true);
			}
			else if(messageID == "xavier_restart") {
				restartXavierNotSync();
				acceptRestart();
			}
			else if(messageID == "parm_reset") {
				parmReset();
			}
			else if(messageID == "parm_close") {
				parmClose();
			}
			else if(messageID == "auto_manual") {
				production_vm->setOperationValue(false);
			}
			else if(messageID == "exit_reset") {
				confirmResetExitPass();
			}
		}
		else{
			if(messageID == "auto_manual"){
				production_vm->setOperationValue(true);
			}
		}
	}).detach();
}

void MasterApp::exitDoubleClicked()
{
	UI_INFO("[MasterApp::exitDoubleClicked]");
	if(config.stateBackupProcess == StateBackupProcess::BUSY)
		modalDialog.getModalDialog(false, "process_busy");
	else
		modalDialog.getModalDialog(true, "exit_click");
}

void MasterApp::updateSpeedSlider(int motor, int value)
{
	switch(motor) {
	case MOTOR_S1:
		config.sliderS1 = value;
		break;
	case MOTOR_S2:
		config.sliderS2 = value;
		break;
	case MOTOR_S3X:
		config.sliderS3x = value;
		break;
	case MOTOR_S3Y:
		config.sliderS3y = value;
		break;
	case MOTOR_S4X:
		config.sliderS4x = value;
		break;
	case MOTOR_S4Y:
		config.sliderS4y = value;
		break;
	case MOTOR_S9X:
		config.sliderS9x = value;
		break;
	case MOTOR_S9Y:
		config.sliderS9y = value;
		break;
	case MOTOR_S9Z:
		config.sliderS9z = value;
		break;
	default:
		break;
	}
	config.writeFileConfig();
}
