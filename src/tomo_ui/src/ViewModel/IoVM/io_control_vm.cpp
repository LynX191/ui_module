#include "io_control_vm.h"
#include <ros/ros.h>

using namespace tomo_peripherals;

IoControlVM::IoControlVM(MasterApp* masterApp, QObject* parent) : master_app(masterApp), client(*masterApp->client), QObject{parent}
{
	_IOThread  = nullptr;
	_ioWriting = false;
	_ioThreadcv.notify_all();
	_outputData.resize(MAX_IO_OUTPUT);
	_inputData.resize(MAX_IO_INPUT);
}

IoControlVM::~IoControlVM()
{
	if(_IOThread)
		close();
}

void IoControlVM::close()
{
	_closeIOThread = true;
	_stateThread   = true;
	_isCompletedI  = false;
	_isCompletedO  = false;
	_ioThreadcv.notify_all();
	if(_IOThread) {
		_IOThread->join();
		_IOThread.reset();
		_IOThread = nullptr;
	}
}

void IoControlVM::setAfterUiCreated()
{
	return;
}

void IoControlVM::changeReceiveSignalMode(bool value)
{
	if(!value){
		close();
		return;
	}
	_closeIOThread = !value;
	if(!_IOThread)
		_IOThread = std::make_shared<std::thread>(&IoControlVM::ioThread, this);
}

void IoControlVM::changeThreadState(bool value)
{
	if(_stateThread == value) return;
	_stateThread = value;
	_ioThreadcv.notify_all();
}

void IoControlVM::sendOutput(int module, int pin, int state)
{
	if(!master_app->serverReady) {
		master_app->modalDialog.getModalDialog(false, "wait_server");
		return;
	}
	if(!master_app->systemReady()) {
		UI_ERROR("[IoControlVM::sendOutput]: Lost communication");
		master_app->modalDialog.getModalDialog(false, "system_lost");
		return;
	}
	if(((pin == 13 && module == 2) || (pin == 15 && module == 2)) && !_inputAccepted && state == 1){
		if(pin == 13)
			master_app->modalDialog.getModalDialog(true, "it_pick");
		if(pin == 15)
			master_app->modalDialog.getModalDialog(true, "it_place");
		return;
	}
	UI_INFO("[IoControlVM::sendOutput] IDO %d-%d: %d", pin, module, state);
	// _outputData[(module - 1) * 16 + (pin - 1)] = state;
	// Q_EMIT outputDataChanged();
	// safetyLock();
	std::thread([this, module, pin, state]() {
		_ioWriting		 = true;
		uint16_t address = (module - 1) * 16 + (pin - 1);
		UI_INFO("[client.wmWriteOutput]");
		client.wmWriteOutput(CMD_SEND_WAIT, address, state);
	}).detach();
}

int IoControlVM::ioThread()
{
	UI_WARN("ioThread: Waiting for server...");
	client.waitForServer();
	UI_WARN("ioThread: Running");
	int skipCount = 0;
	while(!_closeIOThread && master_app->powerUpState()) {
		std::unique_lock<std::mutex> lock(_ioThreadMutex);
		_ioThreadcv.wait(lock, [this] { return _stateThread; });
		// UI_WARN("ioThreadState: %s" , _stateThread ? "Running":"Pause");

		if(_closeIOThread)
			return -1;
		if(_ioWriting) {
			_ioWriting = false;
		}
		else {
			if(skipCount)
				skipCount--;
			else {
				tVectorU outputData, inputData;
				tVectorI inputAnalog;
				if(!master_app->serverReady)
					return -1;
				if(!client.wmReadOutputs(CMD_SEND_WAIT, 0, MAX_IO_OUTPUT, outputData))
					skipCount = 50;
				else {
					tVectorU tempOutputData = outputData;
					updateOutputData(tempOutputData);
					if(!client.wmReadInputs(CMD_SEND_WAIT, 0, MAX_IO_INPUT, inputData))
						skipCount = 50;
					else{
						tVectorU tempInputData = inputData;
						updateInputData(tempInputData);
						safetyLock();
					}
					int distanceUm;
					for(int index = TSTM_PICK_UP_HEIGHT_REACHED; index <= LINE_CLEARANCE; index++) {
						AnalogSignal signal = (AnalogSignal) index;
						if(!client.wmReadAnalogInput(CMD_SEND_WAIT, index, distanceUm))
							skipCount = 50;
						else
							inputAnalog.push_back((int) distanceUm / 1000.0);
					}
					updateInputAnalog(inputAnalog);
				}
			}
		}
		lock.unlock();
		usleep(100000);
	}
	return 0;
}

void IoControlVM::safetyLock()
{
	std::vector<int> privateOnCondition;
	bool newEnable = false;

	// UI_INFO("OutputData size %d, InputData size %d ", _outputData.size(), _inputData.size());
	std::vector<int> newOutputDataCondition(ConditionFromOutput.size());
	for(int i = 0; i < ConditionFromOutput.size(); i++) {
		int order				  = ConditionFromOutput[i].first;
		int data				  = (int) ModuleSignalToAddress[order];
		newOutputDataCondition[i] = _outputData.at(data);
	}
	dataOutputCondition = newOutputDataCondition;
	newEnable			= true;
	for(int i = 0; i < ConditionFromOutput.size(); i++) {
		if(dataOutputCondition[i] == 1){
			privateOnCondition.insert(privateOnCondition.end(), ConditionFromOutput[i].second.begin(), ConditionFromOutput[i].second.end());
		}
	}

	std::vector<int> newInputDataCondition(ConditionFromInput.size());
	for(int i = 0; i < ConditionFromInput.size(); i++){
		int order = ConditionFromInput[i].first;
		int data				  = (int) ModuleSignalToAddress[order];
		newInputDataCondition[i] = _inputData.at(data);
	}
	dataInputCondition = newInputDataCondition;
	newEnable			= true;
	for(int i = 0; i < ConditionFromInput.size(); i++){
		if(dataInputCondition[i] == 1){
			privateOnCondition.insert(privateOnCondition.end(), ConditionFromInput[i].second.begin(), ConditionFromInput[i].second.end());
		}
	}

	if(newEnable){
		QVector<int> newEnableData(MAX_IO_OUTPUT);
		for(int i = 0; i < MAX_IO_OUTPUT; i++) {
			newEnableData[i] = 1;
		}
		for(int i = 0; i < privateOnCondition.size(); i++) {
			if(IOCondition[privateOnCondition[i]].first.size() == std::count(privateOnCondition.begin(), privateOnCondition.end(), privateOnCondition[i])) {
				// UI_INFO("---------------");
				for(int k = 0; k < IOCondition[privateOnCondition[i]].second.size(); k++) {
					std::vector<int> vectorBC = IOCondition[privateOnCondition[i]].second;
					int indexBC				  = vectorBC[k];
					int indexAC				  = ModuleSignalToAddress[indexBC];
					// UI_WARN("Output disable: %s ",ModuleSignalToString[indexBC].c_str());
					newEnableData[indexAC]	  = 0;
				}
			}
			// UI_WARN("Index of vector disable: %d ",privateOnCondition[i]);
		}
		updateOutputEnable(newEnableData);
	}
}

QVector<int> IoControlVM::outputData()
{
	return _outputData;
}

QVector<int> IoControlVM::inputData()
{
	return _inputData;
}

void IoControlVM::inputAccepted(QString input){
	_inputAccepted = true;
	UI_INFO("IoControlVM::inputAccepted");
	if(input == "it_pick") {
		sendOutput(2, 13, 1);
	}
	else if(input == "it_place"){
		sendOutput(2, 15, 1);
	}
	_inputAccepted = false;
}

QVector<int> IoControlVM::outputEnable()
{
	return _outputEnable;
}

void IoControlVM::setFilterData(QString data){
	_filterData = convertToStdString(data);
	if(data == "input") inputCondition = {0,0,0,0};
}

void IoControlVM::setCompletedIO(bool value){
	_isCompletedI  = value;
	_isCompletedO  = value;
}

void IoControlVM::updateOutputData(tVectorU& outputData)
{
	QVector<int> newOutputData;
	tVectorU tempOutputData = outputData;
	std::transform(tempOutputData.begin(), tempOutputData.end(), std::back_inserter(newOutputData),
				   [](unsigned char value) { return static_cast<int>(value); });

	if(_outputData.size() != newOutputData.size() || !std::equal(_outputData.begin(), _outputData.end(), newOutputData.begin())) {
		_outputData = std::move(newOutputData);
		Q_EMIT outputDataChanged();
	}
	if(!_isCompletedO && master_app->sequenceReady) {
		_guardDoorState = _outputData.at(4);
		// UI_INFO("Guard Door State: %s", _guardDoorState? "Lock":"Unlock");
		Q_EMIT master_app->production_vm->emergencyEvent(!_guardDoorState, "guard");
	}
}

void IoControlVM::updateInputData(tVectorU& inputData)
{
	QVector<int> newInputData;
	tVectorU tempInputData = inputData;
	std::transform(tempInputData.begin(), tempInputData.end(), std::back_inserter(newInputData),
				   [](unsigned char value) { return static_cast<int>(value); });

	if(_inputData.size() != newInputData.size() || !std::equal(_inputData.begin(), _inputData.end(), newInputData.begin())) {
		_inputData = std::move(newInputData);
		Q_EMIT inputDataChanged();
	}
	if(_filterData == "input"){
		UI_WARN("IDI 5-3 value: %d, IDI 6-3 value: %d, IDI 7-3 value: %d, IDI 8-3 value: %d, ",_inputData[36], _inputData[37], _inputData[38], _inputData[39]);
		inputCondition = {_inputData[36], _inputData[37], _inputData[38], _inputData[39]};
		if(((_inputData[36] +  _inputData[37]) >= 1 && (_inputData[38] + _inputData[39] >=1)) || isTimeOutInput )
			setInputPnpState(inputCondition[0] == 0 && inputCondition[1] == 1 && inputCondition[2] == 0 && inputCondition[3] == 1);
		master_app->inputCondition = _inputPnpState;
	}

	if(!_isCompletedI && master_app->sequenceReady){
		if(_inputData[4] != _gd1) {
			_gd1 = _inputData[4];
			Q_EMIT master_app->production_vm->emergencyEvent(_gd1, "guard1");
			UI_WARN("Change state guard1: %d", _gd1);
		}
		if(_inputData[5] != _gd2) {
			_gd2 = _inputData[5];
			Q_EMIT master_app->production_vm->emergencyEvent(_gd2, "guard2");
			UI_WARN("Change state guard2: %d", _gd2);
		}
		if(_inputData[6] != _gd3) {
			_gd3 = _inputData[6];
			Q_EMIT master_app->production_vm->emergencyEvent(_gd3, "guard3");
			UI_WARN("Change state guard3: %d", _gd3);
		}
		if(_inputData[7] != _gd4) {
			_gd4 = _inputData[7];
			Q_EMIT master_app->production_vm->emergencyEvent(!_gd4, "cm_door");
			UI_WARN("Change state cm_door: %d", !_gd4);
		}
		if(_inputData[27] != _es1) {
			_es1 = _inputData[27];
			Q_EMIT master_app->production_vm->emergencyEvent(!_es1, "estop1");
			UI_WARN("Change state estop1: %d", !_es1);
		}
		if(_inputData[28] != _es2) {
			_es2 = _inputData[28];
			Q_EMIT master_app->production_vm->emergencyEvent(!_es2, "estop2");
			UI_WARN("Change state estop2: %d", !_es2);
		}
		if(_inputData[30] != _es3) {
			_es3 = _inputData[30];
			Q_EMIT master_app->production_vm->emergencyEvent(!_es3, "estop3");
			UI_WARN("Change state estop3: %d", !_es3);
		}
		if(_inputData[31] != _es4) {
			_es4 = _inputData[31];
			Q_EMIT master_app->production_vm->emergencyEvent(!_es4, "estop4");
			UI_WARN("Change state estop4: %d", !_es4);
		}
		if(_inputData[73] != _es5) {
			_es5 = _inputData[73];
			Q_EMIT master_app->production_vm->emergencyEvent(!_es5, "estop5");
			UI_WARN("Change state estop5: %d", !_es5);
		}
		if(_inputData[74] != _es6) {
			_es6 = _inputData[74];
			Q_EMIT master_app->production_vm->emergencyEvent(!_es6, "estop6");
			UI_WARN("Change state estop6: %d", !_es6);
		}
	}
}

void IoControlVM::updateInputAnalog(tVectorI analogValue)
{
	m_inputAnalogMap.clear();
	for(const int& value : analogValue) {
		m_inputAnalogMap.append(QString::number(value));
	}
	Q_EMIT inputAnalogMapChanged();
}

void IoControlVM::updateOutputEnable(QVector<int> newOutputEnable)
{
	if(_outputEnable.size() != newOutputEnable.size() || !std::equal(_outputEnable.begin(), _outputEnable.end(), newOutputEnable.begin())) {
		_outputEnable = std::move(newOutputEnable);
		Q_EMIT outputEnableChanged();
		// UI_INFO("[IoControlVM::outputEnableChanged]");
	}
}
bool IoControlVM::inputPnpState()
{
	return _inputPnpState;
}
void IoControlVM::setInputPnpState(bool value)
{
	_inputPnpState = value;
	Q_EMIT inputPnpStateChanged(_inputPnpState);
}
void IoControlVM::timeoutInput(bool value){
	isTimeOutInput = value;
}
QStringList IoControlVM::getTitleMap() const
{
	return m_titleList;
}
QVariantMap IoControlVM::getOutputModelMap() const
{
	return m_outputModelMap;
}
QVariantMap IoControlVM::getInputModelMap() const
{
	return m_inputModelMap;
}
QVariantMap IoControlVM::getIOModelMap() const
{
	return m_ioModelMap;
}
QVariantMap IoControlVM::getOutputItemModelMap() const
{
	return m_outputItemModelMap;
}
QVariantMap IoControlVM::getInputItemModelMap() const
{
	return m_inputItemModelMap;
}
QStringList IoControlVM::getInputAnalogMap() const
{
	return m_inputAnalogMap;
}

void IoControlVM::getYamlIOput()
{
	std::string configPath = getenv("HOME") + std::string("/ui_moduleinstall/share/tomo_ui/config/carton_iomap.yaml");
	std::ifstream file(configPath);
	if(file) {
		try {
			yamlData = YAML::LoadFile(configPath);

			if(!yamlData["titleModel"] || !yamlData["ioModel"] || !yamlData["outputItemModel"] ||
			!yamlData["inputItemModel"] || !yamlData["outputListModel"] || !yamlData["inputListModel"] )
			{
				master_app->ioMapConfig = false;
				UI_ERROR("[IoControlVM::getYamlIOput] Error reading YAML file");
				return;
			}

			// Get Title
			QStringList titleMap;
			const YAML::Node& titleModelNode = yamlData["titleModel"];

			for(const auto& entry : titleModelNode) {
				titleMap.append(QString::fromStdString(entry["name"].as<std::string>()));
			}
			m_titleList = titleMap;
			Q_EMIT titleMapChanged();

			// Get IO List have multistate
			QVariantMap ioMap;
			for(int i = 0; i <= yamlData["ioModel"].size(); i++) {
				const std::string key = std::to_string(i);
				if(yamlData["ioModel"][key]) {
					const YAML::Node& itemList = yamlData["ioModel"][key];
					QVariantList itemListVariant;

					for(const auto& entry : itemList) {
						QVariantMap itemMap;
						itemMap["name"]		  = QString::fromStdString(entry["name"].as<std::string>());
						itemMap["orderState"] = QString::fromStdString(entry["orderState"].as<std::string>());
						itemMap["columnN"]	  = entry["columnN"].as<int>();
						itemListVariant.append(itemMap);
					}
					ioMap[QString::number(i)] = itemListVariant;
				}
			}
			m_ioModelMap = ioMap;
			Q_EMIT ioModelMapChanged();

			// Get output item list
			QVariantMap outputItemMap;
			for(int i = 0; i <= 192; i++) {
				const std::string key = std::to_string(i);
				if(yamlData["outputItemModel"][key]) {
					const YAML::Node& itemList = yamlData["outputItemModel"][key];
					QVariantList itemListVariant;

					for(const auto& entry : itemList) {
						QVariantMap itemMap;
						itemMap["name"]	  = QString::fromStdString(entry["name"].as<std::string>());
						itemMap["pin"]	  = entry["pin"].as<int>();
						itemMap["module"] = entry["module"].as<int>();
						itemMap["id"]	  = QString::fromStdString(entry["id"].as<std::string>());
						itemMap["color"]  = "#4ee8ff";
						itemListVariant.append(itemMap);
					}
					outputItemMap[QString::number(i)] = itemListVariant;
				}
			}
			m_outputItemModelMap = outputItemMap;
			Q_EMIT outputItemModelMapChanged();

			// Get input item list
			QVariantMap inputItemMap;
			for(int i = 0; i <= 192; i++) {
				const std::string key = std::to_string(i);
				if(yamlData["inputItemModel"][key]) {
					const YAML::Node& itemList = yamlData["inputItemModel"][key];
					QVariantList itemListVariant;
					for(const auto& entry : itemList) {
						QVariantMap itemMap;
						itemMap["pin"]	  = entry["pin"].as<int>();
						itemMap["module"] = entry["module"].as<int>();
						itemMap["id"]	  = QString::fromStdString(entry["id"].as<std::string>());
						itemMap["color"]  = "#4ee8ff";
						itemListVariant.append(itemMap);
					}
					inputItemMap[QString::number(i)] = itemListVariant;
				}
			}
			m_inputItemModelMap = inputItemMap;
			Q_EMIT inputItemModelMapChanged();

			// Get output list
			QVariantMap outputMap;
			for(int i = 0; i <= yamlData["outputListModel"].size(); i++) {
				const std::string key = std::to_string(i);
				if(yamlData["outputListModel"][key]) {
					const YAML::Node& itemList = yamlData["outputListModel"][key];
					QVariantList itemListVariant;
					for(const auto& entry : itemList) {
						QVariantMap itemMap;
						itemMap["name"]	  = QString::fromStdString(entry["name"].as<std::string>());
						itemMap["module"] = entry["module"].as<int>();
						itemMap["pin"]	  = entry["pin"].as<int>();
						itemMap["color"]  = "#4ee8ff";
						itemMap["id"]	  = QString::fromStdString(entry["id"].as<std::string>());
						itemListVariant.append(itemMap);
					}
					outputMap[QString::number(i)] = itemListVariant;
				}
			}

			m_outputModelMap = outputMap;
			Q_EMIT outputModelMapChanged();
			// Get intput list
			QVariantMap inputMap;
			for(int i = 0; i <= yamlData["inputListModel"].size(); i++) {
				const std::string key = std::to_string(i);
				if(yamlData["inputListModel"][key]) {
					const YAML::Node& itemList = yamlData["inputListModel"][key];
					QVariantList itemListVariant;
					for(const auto& entry : itemList) {
						QVariantMap itemMap;
						itemMap["name"]	  = QString::fromStdString(entry["name"].as<std::string>());
						itemMap["module"] = entry["module"].as<int>();
						itemMap["pin"]	  = entry["pin"].as<int>();
						itemMap["color"]  = "red";
						itemMap["id"]	  = QString::fromStdString(entry["id"].as<std::string>());
						itemListVariant.append(itemMap);
					}
					inputMap[QString::number(i)] = itemListVariant;
				}
			}
			m_inputModelMap = inputMap;
			Q_EMIT inputModelMapChanged();
			UI_INFO("[IoControlVM::getYamlIOput] Initializing I/O from config file '%s'...", configPath.c_str());

			QVector<int> newEnableData(MAX_IO_OUTPUT);
			for(int i = 0; i < MAX_IO_OUTPUT; i++) {
				newEnableData[i] = 1;
			}
			updateOutputEnable(newEnableData);
		}
		catch(const YAML::Exception& e) {
			master_app->ioMapConfig = false;
			UI_ERROR("[IoControlVM::getYamlIOput] Error reading YAML file: %s", e.what());
		}
		catch(const std::exception& e) {
			// Handle other standard exceptions if needed
			master_app->ioMapConfig = false;
			UI_ERROR("[IoControlVM::getYamlIOput] Standard exception: %s", e.what());
		}
		catch(...) {
			// Catch any other unhandled exceptions
			master_app->ioMapConfig = false;
			UI_ERROR("[IoControlVM::getYamlIOput] Unknown error occurred");
		}
		file.close();
	}
	else {
		master_app->ioMapConfig = false;
		UI_ERROR("[IoControlVM::getYamlIOput] Input/Output config file '%s' not found", configPath.c_str());
	}
}
