#include "setting_vm.h"
#include "../../Script/Utilities/utilities.h"
#include <QDir>
#include <QFileInfo>
#include <QFileInfoList>

SettingVM::SettingVM(MasterApp* masterApp, QObject* parent) : master_app(masterApp), QObject(parent)
{
	master_app->config.stateBackupProcess = StateBackupProcess::IDLE;
	backupPath							  = QDir::homePath() + "/tomo_backup/";
	setCamsListID({convertToQString(master_app->config.camerasID[0]), convertToQString(master_app->config.camerasID[1]),
				   convertToQString(master_app->config.camerasID[2])});
}

SettingVM::~SettingVM()
{
}

void SettingVM::setAfterUiCreated()
{
	if(!master_app->isSim()){
	// master_app->xaviersInfoCheck();
		// master_app->xaviersTimeCheck();
	}
}

void SettingVM::saveConfigAs(QString fileName, bool isOverWrite)
{
	if(isOverWrite)
		callRemoveScript(convertToStdString(fileName), true);
	callBackupScript(convertToStdString(fileName));
}

void SettingVM::restoreSavedConfig(QString fileName)
{
	callRestoreScript(convertToStdString(fileName));
}

void SettingVM::resetToDefault()
{
	return;
}

void SettingVM::clearAllBackup()
{
	QDir directory(QDir::homePath() + "/tomo_backup/");
	if(!directory.exists()) {
		UI_WARN("[SettingVM::clearAllBackup] Folder tomo_backup don't exists");
		return;
	}
	QStringList fileNames = directory.entryList(QDir::Files | QDir::NoDotAndDotDot);
	for(const QString& fileName : fileNames) {
		// foreach(const QString& fileName, fileNames) {
		QString filePath = directory.absoluteFilePath(fileName);
		if(QFile::remove(filePath)) {
			UI_WARN("[SettingVM::clearAllBackup] Deleted file: %s", filePath.toStdString().c_str());
		}
		else {
			UI_WARN("[SettingVM::clearAllBackup] Failed to delete file %s:", filePath.toStdString().c_str());
		}
	}
}

void SettingVM::deleteFile(const QString fileName)
{
	callRemoveScript(convertToStdString(fileName), false);
}

void SettingVM::scanLogFilesInFolder()
{
	QDir directory(backupPath);
	QFileInfoList fileList = directory.entryInfoList(QDir::Files);

	for(const QFileInfo& fileInfo : fileList) {
		if(fileInfo.suffix().toLower() == "zip") {
			QString fileName = fileInfo.fileName();
			fileName		 = fileName.left(fileName.lastIndexOf('.'));
			Q_EMIT sendFileName(fileName);
		}
	}
}

void SettingVM::scanLogFilesInFolder2()
{
	QDir directory(QDir::homePath() + "/tomo_backup/");
	if(!directory.exists()) {
		UI_ERROR("[SettingVM::scanLogFilesInFolder] >>> Directory does not exist.");
		QDir().mkdir(QDir::homePath() + "/tomo_backup");
		return;
	}
	QFileInfoList fileList = directory.entryInfoList(QDir::Files);

	for(const QFileInfo& fileInfo : fileList) {
		if(fileInfo.suffix().toLower() == "zip") {
			QString fileName = fileInfo.fileName();
			fileName		 = fileName.left(fileName.lastIndexOf('.'));
			Q_EMIT sendFileName(fileName);
		}
	}
}

void SettingVM::scanDrive()
{	
	QString userName = QString::fromUtf8(std::getenv("SUDO_USER") ? std::getenv("SUDO_USER") : std::getenv("USER"));
	// UI_WARN("Username : %s", convertToStdString(userName).c_str());
	if(userName.isEmpty()) {
		userName = qgetenv("USERNAME");
	}
	QDir directory(QDir::rootPath() + "media/" + userName + "/" );
	QStringList folderList	 = directory.entryList(QDir::Dirs | QDir::NoDotAndDotDot);
	QString currentDriveList = "";
	for(const QString& folderName : folderList) {
		currentDriveList += folderName;
	}
	if(driveList != currentDriveList) {
		driveList = currentDriveList;
		for(const QString& folderName : folderList) {
			Q_EMIT sendDriveName(folderName);
		}
		Q_EMIT driveListChanged();
	}
}

void SettingVM::updateBackupPath(QString drive)
{
	if(drive == "HOME") {
		backupPath = QDir::homePath() + "/tomo_backup/";
	}
	else {
		QString userName = qgetenv("USER");
		if(userName.isEmpty()) {
			userName = qgetenv("USERNAME");
		}
		backupPath = QDir::rootPath() + "media/" + userName + "/" + drive + "/";
	}
}
int SettingVM::enabledIndex0()
{
	return _enabledIndex0;
}
int SettingVM::enabledIndex1()
{
	return _enabledIndex1;
}
int SettingVM::enabledIndex2()
{
	return _enabledIndex2;
}
int SettingVM::enabledIndex3()
{
	return _enabledIndex3;
}
int SettingVM::enabledIndex4()
{
	return _enabledIndex4;
}
int SettingVM::enabledIndex5()
{
	return _enabledIndex5;
}
int SettingVM::enabledIndex6()
{
	return _enabledIndex6;
}
void SettingVM::updatePermissionData(int key, int levelAccess)
{
	int levelAccessCustomer = levelAccess + 1;
	switch(key) {
	case 0:
		master_app->config.controlMasterLevel = levelAccessCustomer;
		break;
	case 1:
		master_app->config.productionLevel = levelAccessCustomer;
		break;
	case 2:
		master_app->config.ioControlLevel = levelAccessCustomer;
		break;
	case 3:
		master_app->config.settingLevel = levelAccessCustomer;
		break;
	case 4:
		master_app->config.systemLogLevel = levelAccessCustomer;
		break;
	case 5:
		master_app->config.parametersLevel = levelAccessCustomer;
		break;
	case 6:{
		master_app->config.exitPassLevel = levelAccessCustomer;
		_enabledIndex6 = levelAccessCustomer;
		break;
	}
	default:
		break;
	}
	master_app->config.writeFileConfig();
}
QString SettingVM::systemLogFont()
{
	return _systemLogFont;
}
int SettingVM::systemLogSize()
{
	return _systemLogSize;
}
int SettingVM::systemLogSpacing()
{
	return _systemLogSpacing;
}
int SettingVM::systemLogLimit()
{
	return _systemLogLimit;
}
int SettingVM::systemLogScroll()
{
	return _systemLogScroll;
}
int SettingVM::freeSpace()
{
	return _freeSpace;
}

void SettingVM::updateFontCmd(QString font)
{
	UI_INFO("[SettingVM::updateFontCmd] %s", convertToStdString(font).c_str());
	master_app->config.textFontCmd = font.toStdString();
	master_app->config.writeFileConfig();
	Q_EMIT systemLogFontChanged();
}
void SettingVM::updateSizeCmd(int size)
{
	UI_INFO("[SettingVM::updateSizeCmd] %d", size);
	master_app->config.textSizeCmd = size;
	master_app->config.writeFileConfig();
	Q_EMIT systemLogSizeChanged();
}
void SettingVM::updateSizeSpacing(int spacing)
{
	UI_INFO("[SettingVM::updateSizeSpacing] %d", spacing);
	master_app->config.lineSpacingCmd = spacing;
	master_app->config.writeFileConfig();
	Q_EMIT systemLogSpacingChanged();
}
void SettingVM::updateLimitLine(int limitLine)
{
	UI_INFO("[SettingVM::updateLimitLine] %d", limitLine);
	master_app->config.limitLineCmd = limitLine;
	master_app->config.writeFileConfig();
	Q_EMIT systemLogLimitChanged();
}
void SettingVM::updateSystemLogScroll(int scrollLine)
{
	UI_INFO("[SettingVM::updateSystemLogScroll] %d", scrollLine);
	master_app->config.scrollLineCmd = scrollLine;
	master_app->config.writeFileConfig();
	Q_EMIT systemLogScrollChanged();
}
void SettingVM::updateFreeSpace(int freeSpace)
{
	_freeSpace = freeSpace;
	UI_INFO("[SettingVM::updateFreeSpace] %d", freeSpace);
	master_app->config.freeSpace = freeSpace;
	master_app->config.writeFileConfig();
	Q_EMIT freeSpaceChanged();
}

QStringList SettingVM::camsListID()
{
	return _camsListID;
}

void SettingVM::setCamsListID(QStringList newList)
{
	_camsListID = newList;
	camsListIDChanged();
}
void SettingVM::updateCamMxId(QString currentID, QString MxId)
{
	auto it = std::find(master_app->config.camerasID.begin(), master_app->config.camerasID.end(),convertToStdString(MxId))	;
	if(it != master_app->config.camerasID.end()){
		master_app->modalDialog.getModalDialog(false, "id_dup");
	}
	std::thread([this, currentID, MxId]() {
		std::string changeCamID = std::string("sudo bash ") + getenv("HOME") + std::string("/ui_moduleinstall/share/tomo_ui/config/command/changeCamID.sh -o " +
															   convertToStdString(currentID) + " -n " + convertToStdString(MxId));
		int returnOfCmd = system(changeCamID.c_str());
		if(returnOfCmd == 0) {
			int findOldCam;
			for(int i = 0; i < master_app->config.camerasID.size(); i++) {
				if(convertToQString(master_app->config.camerasID[i]) == currentID) {
					findOldCam = i;
				}
			}
			_camsListID[findOldCam] = MxId;
			camsListIDChanged();
			master_app->config.camerasID[findOldCam] = convertToStdString(MxId);
			master_app->config.writeFileConfig();
			master_app->modalDialog.getModalDialog(false, "id_success");
		}
		else{
			UI_WARN("Change camera state: %d", returnOfCmd);
			master_app->modalDialog.getModalDialog(false, "id_fail");
		}
	}).detach();
}

void SettingVM::resetToDefaultCamID()
{
	std::thread([this]() {
		std::vector<std::string> defaultList = master_app->config.defaultCamerasID;
		int state;
		for(int i = 0; i < defaultList.size(); i++) {
			std::string changeCamID = std::string("sudo bash ") + getenv("HOME") + std::string("/ui_moduleinstall/share/tomo_ui/config/command/changeCamID.sh -o " +
																   convertToStdString(_camsListID[i]) + " -n " + defaultList[i]);
			int returnOfCmd = system(changeCamID.c_str());
			if(returnOfCmd == 0) {
				state							= 0;
				master_app->config.camerasID[i] = defaultList[i];
				_camsListID[i]					= convertToQString(defaultList[i]);
				camsListIDChanged();
				master_app->config.writeFileConfig();
			}
			else{
				state							= 1;
				break;
			}
		}
		if(state){
			UI_WARN("Change camera state: %d", state);
			master_app->modalDialog.getModalDialog(false, "id_fail");
		}
	}).detach();
}

void SettingVM::updateDefaultCamID()
{
	master_app->config.defaultCamerasID[0] = convertToStdString(_camsListID[0]);
	master_app->config.defaultCamerasID[1] = convertToStdString(_camsListID[1]);
	master_app->config.defaultCamerasID[2] = convertToStdString(_camsListID[2]);
	master_app->config.writeFileConfig();
}

void SettingVM::callBackupScript(std::string name)
{
	if(master_app->config.stateBackupProcess == StateBackupProcess::BUSY) {
		master_app->modalDialog.getModalDialog(false, "backup_process_busy");
		return;
	}

	std::thread([this, name]() {
		std::string cmd = std::string("sudo bash ") + getenv("HOME") + std::string("/ui_moduleinstall/share/tomo_ui/config/command/backup.sh -n ");
		cmd.append(backupPath.toStdString() + name);
		master_app->config.stateBackupProcess = StateBackupProcess::BUSY;
		master_app->modalDialog.getLoadingPopup("Backup on running...");
		if(system(cmd.c_str()) == 0) {
			master_app->config.stateBackupProcess = StateBackupProcess::IDLE;
			master_app->modalDialog.getModalDialog(false, "backup_process_success");
		}
		else
			master_app->config.stateBackupProcess = StateBackupProcess::ERROR;
		master_app->modalDialog.getClosePopupLoading();
		while(master_app->config.stateBackupProcess == StateBackupProcess::BUSY) {
			usleep(500000);
		}
		UI_INFO("Back up process status: %d", master_app->config.stateBackupProcess);
		if(master_app->config.stateBackupProcess == StateBackupProcess::ERROR) {
			master_app->modalDialog.getModalDialog(false, "backup_process_error");
			master_app->config.stateBackupProcess = StateBackupProcess::IDLE;
		}
		Q_EMIT updateList();
	}).detach();
}

void SettingVM::callRemoveScript(std::string name, bool isOverWrite)
{
	std::thread([this, name, isOverWrite]() {
		std::string cmd = std::string("sudo bash ") + getenv("HOME") + std::string("/ui_moduleinstall/share/tomo_ui/config/command/backup.sh -r ");
		cmd.append(backupPath.toStdString() + name);
		master_app->config.stateBackupProcess = StateBackupProcess::BUSY;
		if(system(cmd.c_str()) == 0) {
			if(!isOverWrite) {
				master_app->config.stateBackupProcess = StateBackupProcess::IDLE;
				master_app->modalDialog.getModalDialog(false, "remove_process_success");
			}
		}
		else
			master_app->config.stateBackupProcess = StateBackupProcess::ERROR;
		while(master_app->config.stateBackupProcess == StateBackupProcess::BUSY && !isOverWrite) {
			usleep(500000);
		}
		if(!isOverWrite)
			UI_INFO("Remove process status: %d", master_app->config.stateBackupProcess);

		Q_EMIT updateList();
	}).detach();
}

void SettingVM::callRestoreScript(std::string name)
{
	if(master_app->config.stateBackupProcess == StateBackupProcess::BUSY) {
		master_app->modalDialog.getModalDialog(false, "process_busy");
		return;
	}
	std::string cmd = std::string("sudo bash ") + getenv("HOME") + std::string("/ui_moduleinstall/share/tomo_ui/config/command/recovery.sh -s ");
	cmd.append(backupPath.toStdString() + name);
	int status = system(cmd.c_str());
	// std::thread([this]() { system("pkill -9 tomo_ui"); }).detach();
}

void SettingVM::updateCurrentTime(int index , int value){
	timeVector.resize(6);
	timeVector[index] = value;
	timeConcat = std::to_string(timeVector[0]) + "-" + std::to_string(timeVector[1]) + "-" + std::to_string(timeVector[2]) + " " + std::to_string(timeVector[3]) + ":" + std::to_string(timeVector[4]) + ":" + std::to_string(timeVector[5]);
	// UI_ERROR("Current time: %s", timeConcat.c_str());
}

void SettingVM::startSyncTime(){

	std::thread([this]() {
		master_app->modalDialog.getLoadingPopup("Synchronizing time");
		int checkSuccess;
		std::string cmd = getenv("HOME") + std::string("/ui_moduleinstall/share/tomo_ui/config/command/syncTime.exp");
        cmd.insert(0, "sudo expect ");
		checkSuccess = system(cmd.c_str());
		master_app->modalDialog.getClosePopupLoading();
		if(checkSuccess == 0 ){
			master_app->modalDialog.getModalDialog(false, "sync_time_good");
		}
		else{
			master_app->modalDialog.getModalDialog(false, "sync_time_bad");
		}
		std::this_thread::sleep_for(3000ms);
		openUI();
	}).detach();	
}

void SettingVM::callReturnAction(QString action, QString folderName, int transferred, int total){
	returnAction( action, folderName, transferred, total);
}

void SettingVM::setActionList(int action){
	_currentAction = action;
	std::thread([this](){
		std::string kill_rsync_cmd;
		std::vector<int> xavierList = {1,3};
		switch (_currentAction)
		{
		case 0: // start
			UI_INFO("Start copy installation");
				callReturnAction("start_copy");
			// sshpass -p 'tomo' rsync -avz --progress --timeout=5 ~/ws_tomo/install tomo@192.168.0.161:~/ws_tomo
			for(int i = 0; i < xavierList.size() && _currentAction != 1; i++)
			{
				if(i < 0) i = 0;
				int xavierQueue = xavierList.at(i) - 1;
				std::string xavierName = XAVIER_DEVICES[xavierQueue].name;
				std::string user_host = XAVIER_DEVICES[xavierQueue].user + "@" + XAVIER_DEVICES[xavierQueue].ip;
				std::string password = "'" + XAVIER_DEVICES[xavierQueue].pass + "'";
				std::string remove_command = "sshpass -p "+ password+ " ssh " + user_host + " sudo rm -rf ~/ws_tomo/install" ; // + std::to_string(xavierQueue + 1);
				std::string command = "sshpass -p "+ password+ " sudo rsync -avz --progress --timeout=5 ~/ws_tomo/install " + user_host + ":~/ws_tomo" ; //+ std::to_string(xavierQueue + 1);
				
				UI_INFO("State of remove current installation folder in Xavier %d: %s", xavierList.at(i) , system(remove_command.c_str()) == 0 ? "Success" : "Fail");  
				UI_INFO("Current command terminate:\n%s" , command.c_str());
				int valueReturn = 0;
				std::string output;
				FILE* pipe = popen(command.c_str(), "r");
				if (!pipe) {
					std::cerr << "popen failed!" << std::endl;
            		continue; // Move to the next folder if popen fails
				}
				
				// Read the command's output into a string
				char buffer[128];
				int waitForTransfer, totalToTransfer; 
				bool chk_exist = false, chk_right = false;
				while (!feof(pipe) && valueReturn != EXIT_FAILURE) {
					if (fgets(buffer, 128, pipe) != nullptr) {
						output += buffer;
						std::istringstream output_stream(buffer);
						std::regex progressRegex(R"((xfr#\d+), to-chk=(\d+)/(\d+))");	
						std::smatch matches;
						std::string line;
								
						while (std::getline(output_stream, line)) {
							if (std::regex_search(line, matches, progressRegex)) {
								chk_exist = true;
								std::string transferNumber = matches[1].str();
								waitForTransfer = std::stoi(matches[2].str());
								totalToTransfer = std::stoi(matches[3].str());
								// std::cout << "Transfer " << transferNumber << ": "
								// 		<< waitForTransfer << " out of " << totalToTransfer << " waiting for transfer." << std::endl;
								callReturnAction("file_transferred", convertToQString(xavierName) , totalToTransfer - waitForTransfer, totalToTransfer);
								if(waitForTransfer == 0 && i == xavierList.size() - 1 ){
									callReturnAction("finished_copy");
								}
								chk_right = (waitForTransfer == 0);
							}
							if(!chk_exist) chk_right = true;								
						}
					}
				}

				valueReturn = pclose(pipe);
				if(valueReturn == 0) {
					if(!chk_exist)
						callReturnAction("file_transferred",  convertToQString(xavierName));
					else if(!chk_right)
						callReturnAction("file_transferred",  convertToQString(xavierName), totalToTransfer, totalToTransfer);
				}	
				else if (valueReturn != 0) {
					callReturnAction("transfer_fail",  convertToQString(xavierName));
				}
				if(_currentAction == 3){
					callReturnAction("finished_copy");
					break;
				}
			}
			if(_currentAction != 1)
				callReturnAction("finished_copy");
			break;
        case 1: // pause
            UI_INFO("Pausing copy installation");
			kill_rsync_cmd = "sudo kill $(ps aux | grep '[s]shpass' | awk '{print $2}')";
			if(system(kill_rsync_cmd.c_str()) == 0 ){
				UI_INFO("Rsync process with PID paused successfully");//, pid);
			} else {
				UI_ERROR("Failed to pause rsync process with PID ");//, pid);
			}
            // }
            break;
        case 2: // continue
            UI_INFO("Continuing copy installation");
			// _currentCopyOrder--;
			setActionList(0);
            break;
		case 3: // cancel
			UI_INFO("Cancel copy installation");
			callReturnAction("finished_copy");
			kill_rsync_cmd = "sudo kill $(ps aux | grep '[s]shpass' | awk '{print $2}')";
			if(system(kill_rsync_cmd.c_str()) == 0 ){
				UI_INFO("Rsync process canceled successfully");
			} else {
				UI_ERROR("Failed to cancel rsync process");
			}
			break;
		case 4: //ping failed
			kill_rsync_cmd = "sudo kill $(ps aux | grep '[s]shpass' | awk '{print $2}')";
			if(system(kill_rsync_cmd.c_str()) == 0 ){
				UI_INFO("Transfer failed, connection issue");
			} else {
				UI_ERROR("Failed to stop transfer, connection issue");
			}
			break;
	}
	}).detach();	
}

void SettingVM::updateImaging(QString imagingZipPath, QString imagingZipName, QString imagingName){
	std::thread([this, imagingZipPath, imagingZipName, imagingName ]() {

		std::string user_host = XAVIER_DEVICES[1].user + "@" + XAVIER_DEVICES[1].ip;
		std::string password = "'" + XAVIER_DEVICES[1].pass + "'";
		
		std::string command = "sshpass -p " + password + " rsync -avz --progress --timeout=5 " + convertToStdString(imagingZipPath) + " " + user_host + ":~/tomo_config";
		UI_INFO("Current command terminate:\n%s" , command.c_str());
		int valueReturn = 0;
		std::string output;

		FILE* pipe = popen(command.c_str(), "r");
		if (!pipe) {
			std::cerr << "popen failed!" << std::endl;
		}
		
		// Read the command's output into a string
		char buffer[128];
		int waitForTransfer, totalToTransfer; 
		bool chk_exist = false, chk_right = false;
		while (!feof(pipe) && valueReturn != EXIT_FAILURE) {
			if (fgets(buffer, 128, pipe) != nullptr) {
				output += buffer;
				std::istringstream output_stream(buffer);
				std::string line;
						
				while (std::getline(output_stream, line)) {
					UI_INFO("%s", line.c_str());							
				}
			}
		}

		std::string arg = convertToStdString(imagingZipPath) + " " + convertToStdString(imagingZipName) + " " + convertToStdString(imagingName) + " " + user_host + " " + password;
        std::string cmd = + getenv("HOME") + std::string("/ui_moduleinstall/share/tomo_ui/config/command/updateImaging.exp " + arg);
		cmd.insert(0, std::string("sudo expect "));
		UI_WARN("Current command: %s", cmd.c_str());
		if(system(cmd.c_str()) == 0) {
			UI_WARN("[SettingVM::updateImaging] Imaging update successfully");
		}
		else {
			UI_ERROR("[SettingVM::updateImaging] Imaging update failed");
		}
		Q_EMIT enabledImagingBtn();
	}).detach();
}

void SettingVM::updateControl(QString controlZipPath, QString controlZipName, QString controlName){
	std::thread([this, controlZipPath, controlZipName, controlName ]() {
		if(checkZipFile("tomo_control", convertToStdString(controlZipPath))){
			int result = 0;
			std::vector<int> xavierList = {1, 3, 4};
			for(int i = 0; i < xavierList.size() && _currentAction != 1; i++)
			{
				int xavierQueue = xavierList.at(i) - 1;
				std::string user_host = XAVIER_DEVICES[xavierQueue].user + "@" + XAVIER_DEVICES[xavierQueue].ip;
				std::string password = "'" + XAVIER_DEVICES[xavierQueue].pass + "'";
				
				std::string command = "sshpass -p " + password + " rsync -avz --progress --timeout=5 " + convertToStdString(controlZipPath) + " " + user_host + ":~/ws_tomo/";
				UI_INFO("Current command terminate:\n%s" , command.c_str());
				int valueReturn = 0;
				std::string output;

				FILE* pipe = popen(command.c_str(), "r");
				if (!pipe) {
					std::cerr << "popen failed!" << std::endl;
				}
				
				// Read the command's output into a string
				char buffer[128];
				int waitForTransfer, totalToTransfer; 
				bool chk_exist = false, chk_right = false;
				while (!feof(pipe) && valueReturn != EXIT_FAILURE) {
					if (fgets(buffer, 128, pipe) != nullptr) {
						output += buffer;
						std::istringstream output_stream(buffer);
						std::string line;
								
						while (std::getline(output_stream, line)) {
							UI_INFO("%s", line.c_str());							
						}
					}
				}

				std::string arg = convertToStdString(controlZipPath) + " " + convertToStdString(controlZipName) + " " + convertToStdString(controlName) + " " + user_host + " " + password;
				std::string cmd = + getenv("HOME") + std::string("/ui_moduleinstall/share/tomo_ui/config/command/updateControl.exp " + arg);
				cmd.insert(0, std::string("sudo expect "));
				UI_WARN("Current command: %s", cmd.c_str());
				if(system(cmd.c_str()) != 0) {
					result = 1;
					UI_ERROR("[SettingVM::updateControl] Control update failed");
					break;
				}
			}
			if(result)
				master_app->modalDialog.getModalDialog(false, "control_failed");
			else{
				UI_WARN("[SettingVM::updateControl] Sucessfully update control");
			}
		}
		else{
			master_app->modalDialog.getModalDialog(false, "control_wrong");
		}
		Q_EMIT enabledControlBtn();
	}).detach();
}

void SettingVM::updateUi(QString uiZipPath, QString uiZipName, QString uiName){
	std::thread([this, uiZipPath, uiZipName, uiName ]() {
		if(checkZipFile("tomo_ui", convertToStdString(uiZipPath))){
			int result = 0;			
			std::string command = "sudo rsync -avz --progress --timeout=5 " + convertToStdString(uiZipPath) + " ~/ui_module";
			UI_INFO("Current command terminate:\n%s" , command.c_str());
			int valueReturn = 0;
			std::string output;

			FILE* pipe = popen(command.c_str(), "r");
			if (!pipe) {
				std::cerr << "popen failed!" << std::endl;
			}
			
			// Read the command's output into a string
			char buffer[128];
			int waitForTransfer, totalToTransfer; 
			bool chk_exist = false, chk_right = false;
			while (!feof(pipe) && valueReturn != EXIT_FAILURE) {
				if (fgets(buffer, 128, pipe) != nullptr) {
					output += buffer;
					std::istringstream output_stream(buffer);
					std::string line;
							
					while (std::getline(output_stream, line)) {
						UI_INFO("%s", line.c_str());							
					}
				}
			}

			std::string arg = convertToStdString(uiZipPath) + " " + convertToStdString(uiZipName) + " " + convertToStdString(uiName);
			std::string cmd = + getenv("HOME") + std::string("/ui_moduleinstall/share/tomo_ui/config/command/updateUi.exp " + arg);
			cmd.insert(0, std::string("sudo expect "));
			UI_WARN("Current command: %s", cmd.c_str());
			if(system(cmd.c_str()) != 0) {
				result = 1;
				UI_ERROR("[SettingVM::updateUi] User interface update failed");
			}
			if(result)
				master_app->modalDialog.getModalDialog(false, "ui_failed");
			else{
				UI_WARN("[SettingVM::updateUi] Sucessfully user interface");
			}
			openUI();
		}
		else{
			master_app->modalDialog.getModalDialog(false, "ui_wrong");
		}
		Q_EMIT enabledUiBtn();
	}).detach();
}

bool SettingVM::checkZipFile(std::string typeUpdate, std::string path){
	std::string cmd = std::string("sudo bash ") + getenv("HOME") + std::string("/ui_moduleinstall/share/tomo_ui/config/command/checkZipFile.sh " + path + " " + typeUpdate);
	UI_WARN("[SettingVM::checkZipFile] Current terminate \n %s", cmd.c_str());
	// int valueReturn;
	// std::string output;
	// master_app->terminateReturn(cmd, valueReturn, output);
	bool result = system(cmd.c_str());
	// UI_WARN("[SettingVM::checkZipFile] |%s|", output.c_str());	
	if(result){
		UI_INFO("EXISTS");
	}
	else{
		UI_INFO("NOT EXISTS");
	}
	return result;
}

void SettingVM::openUI()
{
	// Run Ui
	std::thread([this]() {
		std::string cmd = getenv("HOME") + std::string("/tomo_config/openUI.exp");
        cmd.insert(0, "sudo chmod -R 777 ~/ui_moduleinstall * && sudo expect ");
		system(cmd.c_str());
	}).detach();
}