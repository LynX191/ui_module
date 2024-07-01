#include "dash_board_vm.h"

// DashboardVM::DashboardVM(QObject* parent) : QObject{parent}
// #include <QDebug>
DashboardVM::DashboardVM(QObject* parent, MasterApp* masterApp) : QObject{parent}, master_app(masterApp)
{
}

DashboardVM::~DashboardVM()
{
}

void DashboardVM::setTrackviewModels(QHash<QString, QSharedPointer<TrackVM>>& tracks)
{
	QVector<TrackVM*> Tracks;
	Tracks.append(tracks["TomoPose"].data());
	Tracks.append(tracks["TomoEye"].data());
	Tracks.append(tracks["Shipper"].data());
	Tracks.append(tracks["PIM"].data());
	setListTrackVM(Tracks);
	track_view_models.insert("TomoPose", tracks["TomoPose"]);
	track_view_models.insert("TomoEye", tracks["TomoEye"]);
	track_view_models.insert("Shipper", tracks["Shipper"]);
	track_view_models.insert("PIM", tracks["PIM"]);
}

QVector<TrackVM*> DashboardVM::listTrackVM()
{
	return _listTrackVM;
}

QObject* DashboardVM::trackListAt(int index) const
{
	if(index < 0 || index >= _listTrackVM.size())
		return nullptr;
	return _listTrackVM[index];
}

void DashboardVM::setAfterUiCreated()
{
	Q_EMIT setConfigTomOView();
}

void DashboardVM::setListTrackVM(QVector<TrackVM*>& value)
{
	if(_listTrackVM != value) {
		_listTrackVM = value;
		Q_EMIT listTrackVMChanged();
	}
}

bool DashboardVM::isCovered()
{
	return _isCovered;
}

void DashboardVM::setIsCovered(bool value)
{
	if(_isCovered != value) {
		_isCovered = value;
		Q_EMIT isCoveredChanged();
	}
}

void DashboardVM::doubleClickSlots(QString nameDoc)
{
	return;
}

QVariantList DashboardVM::headModel(){
	return _headModel;
}

QVariantList DashboardVM::shipperModel(){
	return _shipperModel;
}

QVariantList DashboardVM::cartonModel(){
	return _cartonModel;
}

QVariantList DashboardVM::exceptModel(){
	return _exceptModel;
}
void DashboardVM::setHeadModel(QVariantList list){
	if(list != _headModel && list.size() > 0){
		_headModel = list;
		headModelChanged();
	}
}

void DashboardVM::setShipperModel(QVariantList list){
	if(list != _shipperModel && list.size() > 0){
		_shipperModel = list;
		shipperModelChanged();
	}
}

void DashboardVM::setCartonModel(QVariantList list){
	if(list != _cartonModel && list.size() > 0){
		_cartonModel = list;
		cartonModelChanged();
	}
}

void DashboardVM::setExceptModel(QVariantList list){
	if(list != _exceptModel && list.size() > 0){
		_exceptModel = list;
		exceptModelChanged();
	}
}

void DashboardVM::setListModel(bool tabChange){
	std::thread([this, tabChange]() {
		UI_INFO("[DashboardVM::setListModel]");
		std::string user_host = XAVIER_DEVICES[1].user + "@" + XAVIER_DEVICES[1].ip;
		std::string password = "'" + XAVIER_DEVICES[1].pass + "'";
		std::string command = "sshpass -p " + password + " ssh " + user_host + " -o ConnectTimeout=5 python3 < ~/ui_moduleinstall/share/tomo_ui/config/command/list_dir.py";
		UI_INFO("Current command terminate: \n%s", command.c_str());
		int valueReturn;
		std::string input;
		master_app->terminateReturn(command, valueReturn, input);
		if(!tabChange && valueReturn != 0){
			master_app->modalDialog.getModalDialog(false, "list_dir_fail");
		}
		std::string head_model = "CartonVision";
		std::string shipper_model = "StimCartoner";
		std::string carton_model = "PimCartonCounting";
		std::string except_model = "exception_images";
		setHeadModel(extractContent(input, head_model));
		setShipperModel(extractContent(input, shipper_model));
		setCartonModel(extractContent(input, carton_model));
		setExceptModel(extractContent(input, except_model));
	}).detach();	
}

QVariantList DashboardVM::extractContent(const std::string& inputText, const std::string& section) {
	QVariantList itemListVariant;
    std::string startMarker = section + "_Start";
    std::string endMarker = section + "_End";

    size_t startIndex = inputText.find(startMarker);
    size_t endIndex = inputText.find(endMarker);
	std::string output;
    if (startIndex != std::string::npos && endIndex != std::string::npos) {
        startIndex += startMarker.length() + 1;
		output = inputText.substr(startIndex, endIndex - startIndex);
    }
	QString outputQml = convertToQString(output);
	UI_INFO("Folder Number of %s: %d", section.c_str(), outputQml.count("\n") - 1);
	for(int i = 0; i < outputQml.count("\n") - 1; i++) {
		QString currentItem = outputQml.split("\n").at(i+1);
		QVariantMap itemMap;
		itemMap["name"] = currentItem.trimmed();
		itemListVariant.append(itemMap);
	}

    return itemListVariant;
}

void DashboardVM::callReturnAction(QString action, QString folderName, int transferred, int total){
	returnAction( action, folderName, transferred, total);
}
void DashboardVM::setSelectList(QString item, bool state){
	if(state){
		_currentSelectList.push_back(item);
	}
	else {
		if(_currentSelectList.indexOf(item) != -1) _currentSelectList.removeOne(item);			
	}
	// qDebug() << "_currentSelectList\\n" << _currentSelectList;
}

void DashboardVM::setDirectory(QString dir){
	QString converted_dir1 = dir.replace("file://","");
	QString converted_dir2 = converted_dir1.replace(" ", "\\ ");
	_currentDirectory = convertToStdString(converted_dir2);
	UI_INFO("Current saving folder: %s", _currentDirectory.c_str());
}

void DashboardVM::setActionList(int action){
	_currentAction = action;
	std::thread([this](){
		std::string kill_rsync_cmd;
		switch (_currentAction)
		{
		case 0: // start
			UI_INFO("Start copy image");
			if(_currentCopyOrder == 0)
				callReturnAction("start_copy");
			if(_currentSelectList.size() == 0){
				callReturnAction("finished_copy");
				_currentCopyOrder = 0;
			}
			for(int i = _currentCopyOrder; i < _currentSelectList.size() && _currentAction != 1; i++)
			{
				if(i < 0) i = 0;
				_currentCopyOrder = i;
				QString currentFolder = _currentSelectList.at(i);
				QString currentParent;
				if(currentFolder.indexOf("/CartonVision/") != -1)
					currentParent = "/CartonVision";
				else if (currentFolder.indexOf("/StimCartoner/") != -1)
					currentParent = "/StimCartoner";
				else if (currentFolder.indexOf("/PimCartonCounting/") != -1)
					currentParent = "/PimCartonCounting";
				else if (currentFolder.indexOf("/exception_images/") != -1)
					currentParent = "/exception_images";
				std::string user_host = XAVIER_DEVICES[1].user + "@" + XAVIER_DEVICES[1].ip;
				std::string password = "'" + XAVIER_DEVICES[1].pass + "'";
				
				std::string command = "sshpass -p "+ password+ " rsync -avz --progress --timeout=5 " + user_host + ":~/tomo_stats" + convertToStdString(currentFolder) +" "+ _currentDirectory + convertToStdString(currentParent) ;
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
								callReturnAction("file_transferred", currentFolder, totalToTransfer - waitForTransfer, totalToTransfer);
								chk_right = (waitForTransfer == 0);
							}
							if(!chk_exist) chk_right = true;								
						}
					}
				}

				valueReturn = pclose(pipe);
				if(valueReturn == 0) {
					if(!chk_exist)
						callReturnAction("file_transferred", currentFolder);
					else if(!chk_right)
						callReturnAction("file_transferred", currentFolder, totalToTransfer, totalToTransfer);

					if(i == _currentSelectList.size() - 1){
						callReturnAction("finished_copy");
						_currentCopyOrder = 0;
					}
					else{
						callReturnAction("done_folder", currentFolder);
					}
				}	
				else if (valueReturn != 0) {
					callReturnAction("transfer_fail", currentFolder);

				}
				if(_currentSelectList.size() > i){
					UI_INFO("Current coppy folder %s, state %d", convertToStdString(_currentSelectList.at(i)).c_str(), valueReturn);
				}
				if(_currentAction == 3){
					callReturnAction("finished_copy");
            		break;
				}
			}
			break;
        case 1: // pause
            UI_INFO("Pausing copy image");
			kill_rsync_cmd = "sudo kill $(ps aux | grep '[s]shpass' | awk '{print $2}')";
			if(system(kill_rsync_cmd.c_str()) == 0 ){
				UI_INFO("Rsync process with PID paused successfully");//, pid);
			} else {
				UI_ERROR("Failed to pause rsync process with PID ");//, pid);
			}
            // }
            break;
        case 2: // continue
            UI_INFO("Continuing copy image");
			// _currentCopyOrder--;
			setActionList(0);
            break;
		case 3: // cancel
			UI_INFO("Cancel copy image");
			callReturnAction("finished_copy");
			kill_rsync_cmd = "sudo kill $(ps aux | grep '[s]shpass' | awk '{print $2}')";
			if(system(kill_rsync_cmd.c_str()) == 0 ){
				UI_INFO("Rsync process canceled successfully");
			} else {
				UI_ERROR("Failed to cancel rsync process");
			}
			_currentCopyOrder = 0;
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

int DashboardVM::checkConnection(int xavier, bool showDialog){
	// std::promise<int> promise; // Create a promise to hold the result
    // std::thread([this, xavier, showDialog, &promise]() { // Capture the promise by reference
        int correctNumber = xavier - 1;
        std::string pingCmd = "sudo oping " + XAVIER_DEVICES[correctNumber].ip + " -c 1 -w 0,4";
        int stateOutput;
        std::string contentOutput;
        master_app->terminateReturn(pingCmd, stateOutput, contentOutput);
        bool state;// = stateOutput == 0;
		if(contentOutput.find("1 received") != string::npos)
		{
			state = true;
			stateOutput = 0;
		}
		else{
			state = false;
			stateOutput = 1912;
		}
        UI_INFO("[DashboardVM::checkConnection] Xavier %d State: %s", xavier, state ? "Connected" : "Disconnected");
        if (!state) {
            if (showDialog)
                master_app->modalDialog.getModalDialog(false, "list_dir_fail");
        }
		return stateOutput;
        // promise.set_value(stateOutput); // Set the value in the promise
    // }).detach();

    // return promise.get_future().get(); // Get the value from the future
}