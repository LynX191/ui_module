#include "production_vm.h"
#include <fstream>
#include <yaml-cpp/yaml.h>

#include "../../Script/Utilities/utilities.h"

using namespace tomo_peripherals;
ProductionVM::ProductionVM(QObject* parent, MasterApp* masterApp) : QObject{parent}, master_app(masterApp)
// ProductionVM::ProductionVM(QObject* parent) : QObject{parent}
{
}

ProductionVM::~ProductionVM()
{
}

void ProductionVM::showStartLot()
{
	Q_EMIT openStartLotConfirm();
	switchToProduction();
}

void ProductionVM::showEndLot()
{
	Q_EMIT openEndLotConfirm();
	switchToProduction();
}

void ProductionVM::showAbortLot()
{
	Q_EMIT openAbortLotConfirm();
	switchToProduction();
}
// Button

void ProductionVM::startLotClicked()
{
	UI_INFO("ProductionVM::startLotClicked");
	master_app->control_comm->sendMessage(CMD_UI + std::to_string(CMD_UI_START_LOT));
	Q_EMIT startLotClickedChanged();
}
void ProductionVM::setStartLotEnabled(bool value)
{
	if(value != _startLotEnabled) {
		_startLotEnabled = value;
		Q_EMIT startLotEnabledChanged();
	}
}
bool ProductionVM::startLotEnabled()
{
	return _startLotEnabled;
}

void ProductionVM::confirmStartLotClicked()
{
	UI_INFO("ProductionVM::confirmStartLotClicked");
	setLotData = _isStartNewLot;
	// writeFileStatistics(false);
	setEnabledForRunLot(false);
	master_app->control_comm->sendMessage(CMD_UI + std::to_string(CMD_UI_START_LOT_CONFIRM) + "^1");
	Q_EMIT confirmStartLotClickedChanged();
}
void ProductionVM::setConfirmStartLotEnabled(bool value)
{
	if(value != _confirmStartLotEnabled) {
		_confirmStartLotEnabled = value;
		Q_EMIT confirmStartLotEnabledChanged();
	}
}

void ProductionVM::setInitialize(bool value)
{
	switchToProduction();
	Q_EMIT initializing(value);
}
bool ProductionVM::confirmStartLotEnabled()
{
	return _confirmStartLotEnabled;
}

void ProductionVM::cancelStartLotClicked()
{
	UI_INFO("ProductionVM::cancelStartLotClicked");
	master_app->control_comm->sendMessage(CMD_UI + std::to_string(CMD_UI_START_LOT_CONFIRM) + "^0");
	Q_EMIT cancelStartLotClickedChanged();
}
void ProductionVM::setCancelStartLotEnabled(bool value)
{
	if(value != _cancelStartLotEnabled) {
		_cancelStartLotEnabled = value;
		Q_EMIT cancelStartLotEnabledChanged();
	}
}

bool ProductionVM::cancelStartLotEnabled()
{
	return _cancelStartLotEnabled;
}

void ProductionVM::endLotClicked(bool isConfirm)
{
	UI_INFO("[ProductionVM::endLotClicked] : %s", isConfirm ? "Confirm" : "Cancel");
	if(isConfirm) {
		master_app->control_comm->sendMessage(CMD_UI + std::to_string(CMD_UI_END_LOT) + "^0");
		setEndLotEnabled(false);
		setEnabledForOpenDoor(true);
	}
	else {
		master_app->control_comm->sendMessage(CMD_UI + std::to_string(CMD_UI_END_LOT) + "^1");
	}

	Q_EMIT endLotClickedChanged();
}

void ProductionVM::endLotPopup()
{
	Q_EMIT openEndLotPopup();
}

void ProductionVM::setEndLotEnabled(bool value)
{
	if(value != _endLotEnabled) {
		_endLotEnabled = value;
		Q_EMIT endLotEnabledChanged();
	}
}

bool ProductionVM::endLotEnabled()
{
	return _endLotEnabled;
}

void ProductionVM::endLotConfirmClicked()
{
	UI_INFO("ProductionVM::endLotConfirmClicked");
	setLotData = false;
	writeFileStatistics(false);
	master_app->control_comm->sendMessage(CMD_UI + std::to_string(CMD_UI_END_LOT) + "^2");
	Q_EMIT endLotConfirmClickedChanged();
}

bool ProductionVM::endLotConfirmEnabled()
{
	return _endLotConfirmEnabled;
}

void ProductionVM::setEndLotConfirmEnabled(bool value)
{
	if(value != _endLotConfirmEnabled) {
		_endLotConfirmEnabled = value;
		Q_EMIT endLotConfirmEnabledChanged();
	}
}

void ProductionVM::abortLotConfirmClicked(bool isConfirm)
{
	UI_INFO("[ProductionVM::abortLotConfirmClicked] Confirm: %s", isConfirm ? "Yes" : "No");
	if(isConfirm) {
		master_app->control_comm->sendMessage(CMD_UI + std::to_string(CMD_UI_ABORT_LOT) + "^1");
	}
	else {
		master_app->control_comm->sendMessage(CMD_UI + std::to_string(CMD_UI_ABORT_LOT) + "^0");
	}
}

void ProductionVM::actionBSTIMChoosen(int value)
{  // 0 for purge, 1 for keep and 2 for confirm after choosen action
	UI_INFO("[ProductionVM::actionBSTIMChoosen] Action for BSTIM: %s", value ? "Keep" : "Purge");
	if(value == 0) {
		master_app->control_comm->sendMessage(CMD_UI + std::to_string(CMD_UI_ENDLOT_PURGE_BSTIM) + "^1");
	}
	else if(value == 1) {
		// Code for Keep function
	}
}

void ProductionVM::auditConveyorClicked()
{
	// Code for audit conveyor button
	UI_INFO("ProductionVM::auditConveyorClicked");
	return;
}

void ProductionVM::setAuditConveyorEnabled(bool value)
{
	if(value != _auditConveyorEnabled) {
		_auditConveyorEnabled = value;
		Q_EMIT auditConveyorEnabledChanged();
	}
}
bool ProductionVM::auditConveyorEnabled()
{
	return _auditConveyorEnabled;
}

void ProductionVM::resetCountClicked()
{
	UI_INFO("ProductionVM::resetCountClicked");
	master_app->control_comm->sendMessage(CMD_UI + std::to_string(CMD_UI_AUDIT_CONVEYOR) + "^-1^0^0");
	Q_EMIT resetCountClickedChanged();
}

void ProductionVM::setResetCountEnabled(bool value)
{
	if(value != _resetCountEnabled) {
		_resetCountEnabled = value;
		Q_EMIT resetCountEnabledChanged();
	}
}

bool ProductionVM::resetCountEnabled()
{
	return _resetCountEnabled;
}

void ProductionVM::startAuditClicked()
{
	UI_INFO("ProductionVM::startAuditClicked");
	setStartAuditEnabled(false);
	master_app->control_comm->sendMessage(CMD_UI + std::to_string(CMD_UI_AUDIT_CONVEYOR) + "^" + std::to_string(_quantityValue) + "^" +
										  std::to_string(_delayTimeValue) + "^0");
	Q_EMIT startAuditClickedChanged();
}

void ProductionVM::setStartAuditEnabled(bool value)
{
	if(value != _startAuditEnabled) {
		_startAuditEnabled = value;
		Q_EMIT startAuditEnabledChanged();
		setEndAuditEnabled(!value);
	}
}

bool ProductionVM::startAuditEnabled()
{
	return _startAuditEnabled;
}

void ProductionVM::endAuditClicked()
{
	UI_INFO("ProductionVM::endAuditClicked");
	master_app->control_comm->sendMessage(CMD_UI + std::to_string(CMD_UI_AUDIT_CONVEYOR) + "^0^0^0");
	Q_EMIT endAuditClickedChanged();
}

void ProductionVM::setEndAuditEnabled(bool value)
{
	if(value != _endAuditEnabled) {
		_endAuditEnabled = value;
		Q_EMIT endAuditEnabledChanged();
	}
}

bool ProductionVM::endAuditEnabled()
{
	return _endAuditEnabled;
}

// Process property
void ProductionVM::setEnabledForRunLot(bool value)
{
	if(value != _enabledForRunLot) {
		_enabledForRunLot = value;
		Q_EMIT enabledForRunLotChanged(value);
	}
}
bool ProductionVM::enabledForRunLot()
{
	return _enabledForRunLot;
}

void ProductionVM::setEnabledForOpenDoor(bool value)
{
	if(value != _enabledForOpenDoor) {
		_enabledForOpenDoor = value;
		Q_EMIT enabledForOpenDoorChanged(value);
	}
}
bool ProductionVM::enabledForOpenDoor()
{
	return _enabledForOpenDoor;
}

void ProductionVM::setFirstRunning(bool value)
{
	if(value != _firstRunning) {
		_firstRunning = value;
		Q_EMIT firstRunningChanged();
	}
}
bool ProductionVM::firstRunning()
{
	return _firstRunning;
}

void ProductionVM::setFileValid(bool value)
{
	if(value != _fileValid) {
		_fileValid = value;
		if(!value)
			switchToProduction();
		Q_EMIT fileValidChanged(value);
	}
}
bool ProductionVM::fileValid()
{
	return _fileValid;
}

// Production Value
void ProductionVM::setRecipeValue(int value)
{
	UI_INFO("ProductionVM::setRecipeValue");
	if(value != _recipeValue) {
		_recipeValue = value;
		if(master_app->control_comm)
			master_app->control_comm->sendMessage("p" + std::to_string(value));
		Q_EMIT recipeValueChanged();
	}
}
int ProductionVM::recipeValue()
{
	return _recipeValue;
}

void ProductionVM::setProductId(int productId)
{
	if(productId != _productId) {
		switch(productId) {
			case 0:
				setProductName("Precision");
				break;
			case 1:
				setProductName("FreshLook");
				break;
			case 2:
				setProductName("Dailies");
				break;
		}
		master_app->config.productId = productId;
		if(master_app->control_comm)
			master_app->control_comm->sendMessage("n" + std::to_string(productId));
		master_app->config.writeFileConfig();
		UI_INFO("[ProductionVM::setProductId] %s %d", convertToStdString(_productName).c_str(), productId);
		_productId = productId;
		Q_EMIT productIdChanged();
	}
}

int ProductionVM::productId()
{
	return _productId;
}

void ProductionVM::setProductName(QString value){
	_productName = value;
	productNameChanged();
}

QString ProductionVM::productName(){
	return _productName;
}

bool ProductionVM::isStartNewLot()
{
	return _isStartNewLot;
}

void ProductionVM::setIsStartNewLot(bool value)
{
	_isStartNewLot = value;
	Q_EMIT isStartNewLotChanged();
}

bool ProductionVM::bypassValue()
{
	return _bypassValue;
}

void ProductionVM::setBypassValue(bool value)
{
	UI_INFO("[ProductionVM::setBypassValue] %s", value ? "On" : "Off");
	if(value != _bypassValue) {
		_bypassValue = value;
		Q_EMIT bypassValueChanged();
	}
	if(!master_app->sequenceReady)
		return;
	master_app->control_comm->sendMessage(CMD_UI + std::to_string(CMD_UI_BYPASS) + "^" + std::to_string(_bypassValue));
}

int ProductionVM::currentModel()
{
	return _currentModel;
}

void ProductionVM::setCurrentModel(int value)
{
	if(value != _currentModel) {
		_currentModel = value;
		Q_EMIT currentModelChanged();
	}
}

tVectorI ProductionVM::endLotProcess()
{
	return _endLotProcess;
}

void ProductionVM::setEndLotProcess(tVectorI processVector)
{
	// if(processVector != _endLotProcess) {
	_endLotProcess = processVector;
	Q_EMIT endLotProcessChanged();
	Q_EMIT endLotProcessConverted(QVector<int>::fromStdVector(_endLotProcess));
	// }
}

bool ProductionVM::dryRunValue()
{
	return _dryRunValue;
}

void ProductionVM::setDryRunValue(bool value)
{
	UI_INFO("[ProductionVM::setDryRunValue] %s", value ? "On" : "Off");
	if(value != _dryRunValue) {
		_dryRunValue = value;
		Q_EMIT dryRunValueChanged();
	}
	if(!master_app->sequenceReady)
		return;
	master_app->control_comm->sendMessage("*" + std::to_string(value ? 2 : 1));
}


bool ProductionVM::operationValue()
{
	return _operationValue;
}

void ProductionVM::setOperationValue(bool value)
{
	if(value != _operationValue) {
		UI_INFO("[ProductionVM::setOperationValue] %s", value ? "Automatic" : "Manual");
		_operationValue = value;
		Q_EMIT operationValueChanged();
	}
	if(!master_app->sequenceReady)
		return;
	// master_app->control_comm->sendMessage("*" + std::to_string(value ? 2 : 1));
}

int ProductionVM::stagesCompleted()
{
	return _stagesCompleted;
}

void ProductionVM::setStagesCompleted(int value)
{
	if(value != _stagesCompleted) {
		_stagesCompleted = value;
		Q_EMIT stagesCompletedChanged();
	}
}

int ProductionVM::quantityValue()
{
	return _quantityValue;
}

void ProductionVM::setQuantityValue(int value)
{
	if(value != _quantityValue) {
		_quantityValue = value;
		if(_quantityValue >= 10)
			_quantityValue = 10;
		else if(_quantityValue < 1)
			_quantityValue = 1;
		Q_EMIT quantityValueChanged();
	}
}

int ProductionVM::actualQuantity()
{
	return _actualQuantity;
}

void ProductionVM::setActualQuantity(int value)
{
	if(value != _actualQuantity) {
		_actualQuantity = value;
		Q_EMIT actualQuantityChanged();
	}
}

int ProductionVM::totalBlowouts()
{
	return _totalBlowouts;
}

void ProductionVM::setTotalBlowouts(int value)
{
	if(value != _totalBlowouts) {
		_totalBlowouts = value;
		Q_EMIT totalBlowoutsChanged();
	}
}

int ProductionVM::totalAudit()
{
	return _totalAudit;
}

void ProductionVM::setTotalAudit(int value)
{
	if(value != _totalAudit) {
		_totalAudit = value;
		Q_EMIT totalAuditChanged();
	}
}

void ProductionVM::muteBtnClicked(bool value)
{
	setSoundState(value);
}

void ProductionVM::setSoundState(bool value)
{
	UI_INFO("[ProductionVM::setSoundState] %s ", value ? "On" : "Off");
	if(value != _soundState) {
		_soundState = value;
		Q_EMIT soundStateChanged();
	}
	if(!master_app->sequenceReady)
		return;
	if(master_app->control_comm) {
		// master_app->control_comm->sendMessage("r1");
		master_app->control_comm->sendMessage(CMD_UI + std::to_string(CMD_UI_SET_MUTE) + "^" + std::to_string(!_soundState));
	}
}

bool ProductionVM::soundState()
{
	return _soundState;
}

// Statistics value
void ProductionVM::readFileStatistics()
{
	try {
		// Check file exits and file empyt
		fileStatistics = getenv("HOME") + std::string("/tomo_stats/lot_statistics.yaml");
		if(!checkPathExists(fileStatistics)) {

			allHistoryCycleTimeData = {};
			allLotData				= {};

			writeFileStatistics(false);
		}
		else {
			YAML::Node rootObj = YAML::LoadFile(fileStatistics);

			allHistoryCycleTimeData = rootObj["History of Cycle Time"].as<std::vector<std::string>>();
			convertToHistoryModel(allHistoryCycleTimeData);
			allLotData = rootObj["Lot Data"].as<std::vector<std::string>>();
			convertToLotDataModel(allLotData);
			if(allHistoryCycleTimeData.empty())
				allHistoryCycleTimeData = std::vector<std::string>();
			if(allLotData.empty())
				allLotData = std::vector<std::string>();
		}
	}
	catch(const std::exception& e) {
		UI_ERROR("[ProductionVM::readFileStatistics] Error when reading the application statistics file: %s", e.what());
	}
}

void ProductionVM::writeFileStatistics(bool saveWay)
{
	YAML::Emitter emitter;
	emitter << YAML::BeginMap;

	if(_completedTray == 0 && checkPathExists(fileStatistics) && !setLotData)
		return;

	if(saveWay) {
		insertHistoryData();
	}
	emitter << YAML::Key << "History of Cycle Time" << YAML::Value << allHistoryCycleTimeData;
	convertToHistoryModel(allHistoryCycleTimeData);

	insertLotData(setLotData);
	emitter << YAML::Key << "Lot Data" << YAML::Value << allLotData;
	convertToLotDataModel(allLotData);

	// Write the updated content back to the file
	std::ofstream outFile(fileStatistics);
	if(!outFile.is_open()) {
		UI_ERROR("[ProductionVM::writeFileStatistics] Fail to open the file");
		return;
	}
	outFile << emitter.c_str();
}

void ProductionVM::insertHistoryData()
{
	master_app->xaviersInfoCheck();

	std::string checkStartTime;
	if(_lotStartTimeSec != 0) {
		// Unix time
		int unixTime = _lotStartTimeSec;  // Example Unix time

		// Convert Unix time to time_t
		time_t unixTime_t = static_cast<time_t>(unixTime);

		// Convert Unix time to struct tm
		struct tm* timeinfo;
		timeinfo = localtime(&unixTime_t);

		// Convert struct tm to string
		char buffer[80];
		strftime(buffer, 80, "%Y-%m-%d\n%H:%M:%S", timeinfo);
		checkStartTime = buffer;
	}
	else {
		return;
	}
	// UI_INFO("Current color %s  ", current_color.c_str());
	std::string concatString = "ps  " + std::to_string(_recipeValue) + "|lst  " + checkStartTime + "|ct  " +
							   convertToStdString(_cycleTime) + "|cpm  " + std::to_string(_cartonPerMin) + "|act  " +
							   std::to_string(_averageCycleTime) + "|acp  " + std::to_string(_avgCartonPerMin) + "|nct  " +
							   std::to_string(_completedTray) + "|nlc  " + std::to_string(_noLoadedCarton);
	std::string getPreviousNct = "0", getPreviousLst = "00:00:00";
	if(allHistoryCycleTimeData.size() > 0) {
		size_t findBeginNct = allHistoryCycleTimeData[0].find("nct");
		size_t findEndNct	= allHistoryCycleTimeData[0].find("|nlc");
		getPreviousNct		= allHistoryCycleTimeData[0].substr(findBeginNct + 5, findEndNct - findBeginNct - 5);

		size_t findBeginLst = allHistoryCycleTimeData[0].find("lst");
		size_t findEndLst	= allHistoryCycleTimeData[0].find("|ct");
		getPreviousLst		= allHistoryCycleTimeData[0].substr(findBeginLst + 5, findEndLst - findBeginLst - 5);
	}
	if(_completedTray != stoi(getPreviousNct) || checkStartTime != getPreviousLst) {
		if(allHistoryCycleTimeData.size() >= 200)
			allHistoryCycleTimeData.pop_back();

		std::string current_color = color_map[0];
		if(allHistoryCycleTimeData.size() > 0) {
			auto& entry	  = allHistoryCycleTimeData[0];
			current_color = entry.substr(entry.find("color") + 7, entry.find("|", entry.find("color") + 5) - entry.find("color") - 7);
			if(checkStartTime != getPreviousLst) {
				if(current_color == color_map[0]) {
					last_color = color_map[1];
				}
				else {
					last_color = color_map[0];
				}
				current_color = last_color;
			}
		}
		concatString = concatString + "|color  " + current_color;
		allHistoryCycleTimeData.insert(allHistoryCycleTimeData.begin(), concatString);
	}
}

QVariantList ProductionVM::historyModel() const
{
	return _historyModel;
}

void ProductionVM::convertToHistoryModel(std::vector<std::string> historyVector)
{
	QVariantList itemListVariant;
	for(const auto& entry : historyVector) {
		QVariantMap itemMap;
		itemMap["packSize"] =
			convertToQString(entry.substr(entry.find("ps") + 4, entry.find("|", entry.find("ps") + 2) - entry.find("ps") - 4));
		itemMap["startTime"] =
			convertToQString(entry.substr(entry.find("lst") + 5, entry.find("|", entry.find("lst") + 3) - entry.find("lst") - 5));
		itemMap["cycleTime"] =
			convertToQString(entry.substr(entry.find("ct") + 4, entry.find("|", entry.find("ct") + 2) - entry.find("ct") - 4));
		itemMap["cartonPerMin"] =
			convertToQString(entry.substr(entry.find("cpm") + 5, entry.find("|", entry.find("cpm") + 3) - entry.find("cpm") - 5));
		itemMap["averageCycleTime"] =
			convertToQString(entry.substr(entry.find("act") + 5, entry.find("|", entry.find("act") + 3) - entry.find("act") - 5));
		itemMap["averageCartonPerMin"] =
			convertToQString(entry.substr(entry.find("acp") + 5, entry.find("|", entry.find("acp") + 3) - entry.find("acp") - 5));
		itemMap["completedTrays"] =
			convertToQString(entry.substr(entry.find("nct") + 5, entry.find("|", entry.find("nct") + 3) - entry.find("nct") - 5));
		itemMap["loadedCartons"] =
			convertToQString(entry.substr(entry.find("nlc") + 5, entry.find("|", entry.find("nlc") + 3) - entry.find("nlc") - 5));
		itemMap["color"] =
			convertToQString(entry.substr(entry.find("color") + 7, entry.find("|", entry.find("color") + 5) - entry.find("color") - 7));
		itemListVariant.append(itemMap);
	}
	_historyModel = itemListVariant;
	Q_EMIT historyModelChanged();
}

void ProductionVM::insertLotData(bool checkSetInsert)
{
	std::string checkStartTime, checkEndTime;
	if(_lotStartTimeSec != 0) {
		// Unix time
		int unixTime = _lotStartTimeSec;  // Example Unix time

		// Convert Unix time to time_t
		time_t unixTime_t = static_cast<time_t>(unixTime);

		// Convert Unix time to struct tm
		struct tm* timeinfo;
		timeinfo = localtime(&unixTime_t);

		// Convert struct tm to string
		char buffer[80];
		strftime(buffer, 80, "%Y-%m-%d\n%H:%M:%S", timeinfo);
		checkStartTime = buffer;
	}
	else {
		return;
	}
	if(_lotEndTime != "") {
		checkEndTime = getCurrentTime(true).substr(0, 10) + "\n" + convertToStdString(_lotEndTime);
	}
	else {
		checkEndTime = convertToStdString(_lotEndTime);
	}
	std::string concatString = "ps  " + std::to_string(_recipeValue) + "|lst  " + checkStartTime + "|let  " + checkEndTime + "|tr  " +
							   convertToStdString(_totalRuntime) + "|td  " + convertToStdString(_totalDowntime) + "|cpm  " +
							   std::to_string(_avgCartonPerMin) + "|act  " + std::to_string(_averageCycleTime) + "|nct  " +
							   std::to_string(_completedTray) + "|nlc  " + std::to_string(_noLoadedCarton);
	
	std::string getPreviousLst = "2024-00-00\n00:00:00";
	if(allLotData.size() > 0) {
		size_t findBeginLst = allLotData[0].find("lst");
		size_t findEndLst	= allLotData[0].find("|let");
		getPreviousLst		= allLotData[0].substr(findBeginLst + 5, findEndLst - findBeginLst - 5);
	}

	checkSetInsert = checkStartTime != getPreviousLst;
	if(checkSetInsert) {  // insert new line
		if(allLotData.size() >= 200)
			allLotData.pop_back();
		allLotData.insert(allLotData.begin(), concatString);
	}
	else {	// replace value in line
		if(allLotData.size() > 0)
			allLotData[0] = concatString;
	}
}

QVariantList ProductionVM::lotDataModel() const
{
	return _lotDataModel;
}

void ProductionVM::convertToLotDataModel(std::vector<std::string> historyVector)
{
	QVariantList lotListVariant;

	for(const auto& entry : historyVector) {
		QVariantMap lotMap;
		lotMap["packSize"] =
			convertToQString(entry.substr(entry.find("ps") + 2, entry.find("|", entry.find("ps") + 2) - entry.find("ps") - 2));
		lotMap["startTime"] =
			convertToQString(entry.substr(entry.find("lst") + 3, entry.find("|", entry.find("lst") + 3) - entry.find("lst") - 3));
		lotMap["endTime"] =
			convertToQString(entry.substr(entry.find("let") + 3, entry.find("|", entry.find("let") + 3) - entry.find("let") - 3));
		lotMap["runTime"] =
			convertToQString(entry.substr(entry.find("tr") + 2, entry.find("|", entry.find("tr") + 2) - entry.find("tr") - 2));
		lotMap["downTime"] =
			convertToQString(entry.substr(entry.find("td") + 2, entry.find("|", entry.find("td") + 2) - entry.find("td") - 2));
		lotMap["averageCartonPerMin"] =
			convertToQString(entry.substr(entry.find("cpm") + 3, entry.find("|", entry.find("cpm") + 3) - entry.find("cpm") - 3));
		lotMap["averageCycleTime"] =
			convertToQString(entry.substr(entry.find("act") + 3, entry.find("|", entry.find("act") + 3) - entry.find("act") - 3));
		lotMap["completedTrays"] =
			convertToQString(entry.substr(entry.find("nct") + 3, entry.find("|", entry.find("nct") + 3) - entry.find("nct") - 3));
		lotMap["loadedCartons"] =
			convertToQString(entry.substr(entry.find("nlc") + 3, entry.find("|", entry.find("nlc") + 3) - entry.find("nlc") - 3));
		lotListVariant.append(lotMap);
	}

	_lotDataModel = lotListVariant;
	Q_EMIT lotDataModelChanged();
}

void ProductionVM::addToHistory()
{
	writeFileStatistics(true);
}
int ProductionVM::noBottomShipper()
{
	return _noBottomShipper;
}

void ProductionVM::setNoBottomShipper(int value)
{
	if(value != _noBottomShipper) {
		_noBottomShipper = value;
		Q_EMIT noBottomShipperChanged();
	}
}

int ProductionVM::noTopShipper()
{
	return _noTopShipper;
}

void ProductionVM::setNoTopShipper(int value)
{
	if(value != _noTopShipper) {
		_noTopShipper = value;
		Q_EMIT noTopShipperChanged();
	}
}

QString ProductionVM::lotStartTime()
{
	return _lotStartTime;
}

void ProductionVM::setLotStartTime(QString value)
{
	_lotStartTime = value;
	Q_EMIT lotStartTimeChanged();
}

int ProductionVM::lotStartTimeSec()
{
	return _lotStartTimeSec;
}

void ProductionVM::setLotStartTimeSec(int value)
{
	_lotStartTimeSec = value;
	Q_EMIT lotStartTimeSecChanged();
}

QString ProductionVM::lotEndTime()
{
	return _lotEndTime;
}

void ProductionVM::setLotEndTime(QString value)
{
	_lotEndTime = value;
	setLotData	= false;
	writeFileStatistics(false);
	Q_EMIT lotEndTimeChanged();
}

QString ProductionVM::totalDowntime()
{
	return _totalDowntime;
}

void ProductionVM::setTotalDowntime(QString value)
{
	if(value != _totalDowntime) {
		_totalDowntime = value;
		Q_EMIT totalDowntimeChanged();
	}
}

int ProductionVM::getCurrentTimeProduct()
{
	return getCurrentSec();
}

QString ProductionVM::totalRuntime()
{
	return _totalRuntime;
}

void ProductionVM::setTotalRuntime(QString value)
{
	_totalRuntime = value;
	Q_EMIT totalRuntimeChanged();
}

QString ProductionVM::lotDuration()
{
	return _lotDuration;
}

void ProductionVM::setLotDuration(QString value)
{
	if(value != _lotDuration) {
		_lotDuration = value;
		Q_EMIT lotDurationChanged();
	}
}

QString ProductionVM::cycleTime()
{
	return _cycleTime;
}

void ProductionVM::setCycleTime(QString value)
{
	if(value != _cycleTime) {
		_cycleTime = value;
		Q_EMIT cycleTimeChanged();
	}
}

int ProductionVM::averageCycleTime()
{
	return _averageCycleTime;
}

void ProductionVM::setAverageCycleTime(int value)
{
	if(value != _averageCycleTime) {
		_averageCycleTime = value;
		Q_EMIT averageCycleTimeChanged();
	}
}

int ProductionVM::completedTray()
{
	return _completedTray;
}

void ProductionVM::setCompletedTray(int value)
{
	setLotData	   = false;
	_completedTray = value;
	Q_EMIT completedTrayChanged();
}

int ProductionVM::noLoadedCarton()
{
	return _noLoadedCarton;
}

void ProductionVM::setNoLoadedCarton(int value)
{
	if(value != _noLoadedCarton) {
		_noLoadedCarton = value;
		Q_EMIT noLoadedCartonChanged();
	}
}

int ProductionVM::noIncomingCarton()
{
	return _noIncomingCarton;
}

void ProductionVM::setNoIncomingCarton(int value)
{
	if(value != _noIncomingCarton) {
		_noIncomingCarton = value;
		Q_EMIT noIncomingCartonChanged();
	}
}

int ProductionVM::cartonPerMin()
{
	return _cartonPerMin;
}

void ProductionVM::setCartonPerMin(int value)
{
	if(value != _cartonPerMin) {
		_cartonPerMin = value;
		Q_EMIT cartonPerMinChanged();
	}
}

int ProductionVM::avgCartonPerMin()
{
	return _avgCartonPerMin;
}

void ProductionVM::setAvgCartonPerMin(int value)
{
	if(value != _avgCartonPerMin) {
		_avgCartonPerMin = value;
		Q_EMIT avgCartonPerMinChanged();
	}
}

void ProductionVM::setCurrentDateTime(QString value)
{
	if(value != _currentDateTime) {
		_currentDateTime = value;
		Q_EMIT currentDateTimeChanged();
	}
}

QString ProductionVM::currentDateTime()
{
	return _currentDateTime;
}
