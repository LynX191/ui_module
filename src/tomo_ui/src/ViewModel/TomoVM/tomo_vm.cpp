#include "tomo_vm.h"
#include "../../Dialog/modal_dialogbox.h"
#include "../../Script/Config/cfg_app.h"
#include "../../Script/Utilities/utilities.h"
#include "../ProductionVM/production_vm.h"
#include "tomo_utils/tomo_utils.h"
#include <boost/algorithm/string.hpp>
#include <tomo_devices_carton/def_server.h>
#include <yaml-cpp/yaml.h>

using namespace tomo_peripherals;
bool TomoVM::parseConfigFile()
{
	std::string rvizPath = getenv("HOME") + std::string("/tomo_config/moveit_ui.rviz");
	if(!checkPathExists(rvizPath)) {
		UI_ERROR("Moveit config file '%s' not found", rvizPath.c_str());
		master_app->rvizConfig = false;
	}

	std::string configPath = getenv("HOME") + std::string("/tomo_config/carton_sm.yaml");
	if(!checkPathExists(configPath)) {
		UI_ERROR("State machine config file '%s' not found", configPath.c_str());
		master_app->smConfig = false;
		return false;
	}
	else {
		UI_INFO("Initializing GUI from config file '%s'...", configPath.c_str());
	}

	try {
		modes.clear();
		// states.clear();
		YAML::Node yamlFile = YAML::LoadFile(configPath);
		if(!yamlFile["modes"]) {
			UI_ERROR("[TomoVM::parseConfigFile] Failed");
			master_app->smConfig = false;
			return false;
		}
		for(unsigned i = 0; i < yamlFile["modes"].size(); i++)
			modes.push_back(yamlFile["modes"]["m" + std::to_string(i)].as<std::string>());
		// for(unsigned i = 0; i < yamlFile["states"].size(); i++)
		// 	states.push_back(yamlFile["states"]["s" + std::to_string(i)].as<std::string>());
	}
	catch(...) {
		master_app->smConfig = false;
		UI_ERROR("[TomoVM::parseConfigFile] Failed");
		return false;
	}
	UI_INFO("[TomoVM::parseConfigFile] Done");

	return true;
}

/// Communication from sequence; Udupa; Oct'2023
int TomoVM::commThread()
{
	if(!master_app->control_comm) {
		UI_ERROR("TomoVM.commThread: Comm node is invalid");
		return 0;
	}

	while(!isClosing) {
		UI_INFO("TomoVM.commThread: Waiting for message from SM");
		std::string message = master_app->control_comm->receiveMessage();
		// boost::algorithm::to_lower(message);

		if(!message.empty()) {
			std::vector<std::string> splits;
			boost::split(splits, message, boost::is_any_of("|"));
			for(auto& input : splits) {
				if(input.empty())
					continue;

				int val	 = -1;
				char cmd = input[0];
				if(input.size() > 1 && cmd >= '#' && cmd <= 'z') {
					try {
						val = std::stoi(input.substr(1));
					}
					catch(...) {
						UI_ERROR("Command from SM: Invalid mode value %s\n", input.c_str());
						continue;
					}
					UI_INFO("Command from SM: %c = %d", cmd, val);

					if(cmd == 'p')
						production_vm->setRecipeValue(val);
					if(cmd == 'v' || cmd == 'a')
						val = -val;
					else if(cmd == 'e') {
					}
					else if(cmd == CMD_PARM[0]) {
						processParameters(input.substr(1));
					}
					else if(cmd == CMD_ERROR[0]) {
						processErrorMessage(input.substr(1));
					}
					else if(cmd == CMD_UI[0] && input.size() > 3) {
						processStatistics(input.substr(3));
					}
					setModeTomo(input.substr(0, 1), val);
				}
			}
		}
	}

	return 0;
}

/// Parse sequence message; Udupa; Dec'2023
void TomoVM::parseMessage(const std::string& message, tVectorS& messages, char messagePrefix)
{
	std ::vector<std::string> splits;
	boost::split(splits, message, boost::is_any_of("^"));
	for(auto& split : splits) {
		if(messagePrefix) {
			if(split[0] == messagePrefix)
				messages.push_back(split.substr(1));
		}
		else
			messages.push_back(split);
	}
}

void TomoVM::parseMessage(const std::string& message, tVectorS& messages, tVectorI& parms)
{
	std ::vector<std::string> splits;
	boost::split(splits, message, boost::is_any_of("^"));
	for(auto& split : splits) {
		if(split[0] == '*') {
			if(split.size() > 1)
				messages.push_back(split.substr(1));
		}
		else {
			int val = stringToInt(split);
			if(val == INT_MAX) {
				val = 0;
				UI_ERROR("Command from SM: Invalid message value %s\n", split.c_str());
			}
			parms.push_back(val);
		}
	}
}

/// Process statistical messages from sequence; Udupa; Dec'2023
void TomoVM::processStatistics(const std::string& stats)
{
	tVectorS messages;
	parseMessage(stats, messages, '*');
	bool addEvent = false;
	if(messages.size() % 2) {
		UI_ERROR("Invalid statistical message length (%d) received: %s", (int) messages.size(), stats.c_str());
		return;
	}

	std::string statRef, statData;
	int statId, data;
	for(int i = 0; i < messages.size(); i += 2) {
		statRef = messages[i];
		if(!LotStatRefToId.count(statRef)) {
			UI_ERROR("Invalid statistical data received (%s); Ignoring", statRef.c_str());
			continue;
		}
		statId				  = LotStatRefToId[statRef];
		std::string& statData = messages[i + 1];
		UI_INFO("Setting lot stat data %s = %s", statRef.c_str(), statData.c_str());

		switch(statId) {
		case PROCESS_STATE:
			data = stringToInt(statData);
			/// Show process state here: Udupa; Dec'2023
			setProcessState(convertToQString(ProcessStateToString[data]));
			production_vm->setStartLotEnabled(data == PROCESS_ONLINE);
			production_vm->setEnabledForOpenDoor(data != PROCESS_RUNNING);
			if(data == PROCESS_QUIT) {
				master_app->setSystemReady(false);
			}
			// if(data == PROCESS_ONLINE){
			// 	production_vm->initialized();
			// }
			break;
		case LOT_STATE:
			data = stringToInt(statData);
			/// Show lot state here: Udupa; Dec'2023
			setLotState(convertToQString(LotStateToString[data]));
			break;
		case END_LOT_CHECKLIST: {
			data = stringToInt(statData);
			tVectorI endLotChecklist(ENDLOT_CHECKLIST_COUNT);
			UI_WARN("EndLotChecklist = 0X%X", data);
			for(int i = 0; i < ENDLOT_CHECKLIST_COUNT; i++) {
				endLotChecklist[i] = ((data >> i) & 1);
				UI_WARN("EndLotChecklist[%d] = %d", i, endLotChecklist[i]);
			}
			production_vm->setEndLotProcess(endLotChecklist);
			break;
		}
		case LOT_CARTON_COUNT:
			data = stringToInt(statData);
			production_vm->setNoLoadedCarton(data);
			break;
		case LOT_TOP_SHIPPER_COUNT:
			data = stringToInt(statData);
			production_vm->setNoTopShipper(data);
			break;
		case LOT_BASE_SHIPPER_COUNT:
			data = stringToInt(statData);
			production_vm->setNoBottomShipper(data);
			break;
		case LOT_FILLED_SHIPPER_COUNT:
			data = stringToInt(statData);
			if(data != production_vm->_completedTray) {
				addEvent = true;
				production_vm->setCompletedTray(data);
			}
			break;
		// case LOT_CARTONS_PER_SHIPPER:
		// 	data = stringToInt(statData);
		// 	break;
		case LOT_CARTON_PER_MIN:
			data = stringToInt(statData);
			production_vm->setCartonPerMin(data);
			break;
		case LOT_CARTON_PER_MIN_AVG:
			data = stringToInt(statData);
			production_vm->setAvgCartonPerMin(data);
			break;
		case LOT_START_TIME_SEC:
			data = stringToInt(statData);
			production_vm->setLotStartTimeSec(data);
			break;
		case LOT_START_TIME:
			production_vm->setLotStartTime(convertToQString(statData));
			break;
		case LOT_END_TIME:
			production_vm->setLotEndTime(convertToQString(statData));
			break;
		case LOT_CYCLE_TIME:
			production_vm->setCycleTime(convertToQString(statData));
			break;
		case LOT_CYCLE_TIME_AVG:
			data = stringToInt(statData);
			production_vm->setAverageCycleTime(data);
			break;
		case LOT_DURATION:
			production_vm->setTotalRuntime(convertToQString(statData));
			break;
		case LOT_DOWNTIME:
			production_vm->setTotalDowntime(convertToQString(statData));
			break;
		case AUDIT_SAMPLES:
			// Number of blow outs in current audit; Udupa; 19Dec'23
			data = stringToInt(statData);
			if(data >= 0)
				production_vm->setActualQuantity(data);
			else {
				UI_WARN("Audit sampling finished");
				int totalAudit = production_vm->_totalAudit + 1;
				production_vm->setTotalAudit(totalAudit);
				production_vm->setStartAuditEnabled(true);
				// End audit here; Udupa; 19Dec'23
			}
			break;
		case AUDIT_TOTAL:
			// Total number of blow outs in the lot; Udupa; 19Dec'23
			data = stringToInt(statData);
			production_vm->setTotalBlowouts(data);
			break;
		case AUDITS:
			// Total number of times audit has been started; Update this value where necessary; Udupa; 19Dec'23
			data = stringToInt(statData);
			// production_vm->setTotalAudit(data);
			break;
		default:
			UI_ERROR("Invalid stat id %s = %s", statRef.c_str(), statData.c_str());
			break;
		}
	}
	if(addEvent) {
		production_vm->addToHistory();
		addEvent = false;
	}
}

/// Process error messages from sequence; Udupa; Dec'2023
void TomoVM::processErrorMessage(const std::string& errorMessage)
{
	tVectorI parms;
	tVectorS messages;

	parseMessage(errorMessage, messages, parms);
	if(parms.size() && messages.size()) {
		UI_WARN("Process message received (%d: %s)", parms[0], messages[0].c_str());
		if(setProductionControl(parms[0], messages[0]) < 0)
			master_app->modalDialog.showErrorMessage(parms[0], messages[0]);
	}
	else
		UI_ERROR("Invalid process message received (%s)", errorMessage.c_str());
}

/// Parameters from sequence; Udupa; May'2024
void TomoVM::processParameters(const std::string& parmMessage)
{
	tVectorS parmList;
	parseMessage(parmMessage, parmList, 0);
	if(parmList.size() < 2) {
		UI_ERROR("Invalid parameter list received (ListSize=%d):\n%s", (int) parmList.size(), parmMessage.c_str());
		return;
	}

	int moduleId  = stringToInt(parmList[0]);
	int parmCount = stringToInt(parmList[1]);
	parmList.erase(parmList.begin(), parmList.begin() + 2);

	UI_INFO("processParameters: ModuleID %d; %d parameters", moduleId, parmCount);
	QStringList qStringList;
	for(auto& parm : parmList){
		UI_INFO("    %s", parm.c_str());
		qStringList << convertToQString(parm);
	}

	if(parmList.size() && parmList.size() != parmCount) {
		UI_ERROR("Invalid parameter list received (ListSize=%d; Count=%d):\n%s", (int) parmList.size(), parmCount, parmMessage.c_str());
		return;
	}

	/// Call QML function enable/disable parameters button based on parmCount, OR
	master_app->station_vm->setEnabledParmButton(parmCount > 0);
	/// Call QML function to show parameter list based on parmList
	if(qStringList.size() > 0)
		master_app->station_vm->setParametersList(qStringList);
}

int TomoVM::launchCommNode()
{
	UI_INFO("[TomoVM::launchCommNode] ******** Starting comm node");

	master_app->control_comm = std::make_shared<TopicString>();
	master_app->control_comm->initPublisher("/tomo_ui/comm_control");

	master_app->control_comm->initSubscriber("/tomo_control/comm_ui");
	UI_INFO("[TomoVM::launchCommNode] Comm node started ********\n\n\n\n");

	isClosing	= false;
	comm_thread = std::make_shared<std::thread>(&TomoVM::commThread, this);

	ros::spin();

	return 0;
}

TomoVM::TomoVM(int& argc, char** argv, MasterApp* masterApp, QApplication* qapp, QObject* parent)
	: master_app{masterApp}, m_qapp(qapp), m_argc(argc), m_argv(argv), QObject{parent}
{
	comm_node_thread   = nullptr;
	comm_thread		   = nullptr;
	_isRvizFullScreen  = false;
	_isRvizCalled	   = false;
	_isRvizReadyToCall = false;
	_isSuperUserAct	   = false;
	if(master_app) {
		production_vm	 = master_app->production_vm.data();
		comm_node_thread = std::make_shared<std::thread>(&TomoVM::launchCommNode, this);

		// create track
		QVector<TrackVM*> Tracks;
		Tracks.append(master_app->track_view_models["TomoPose"].data());
		track_view_models.insert("TomoPose", master_app->track_view_models["TomoPose"]);
		setListTrackVM(Tracks);
	}

	// Load mode and states in config file
	if(!parseConfigFile()) {
		modes.push_back("Cannot Parse Config file");
		// states.push_back("Cannot Parse Config file");
	}
	// state and mode inspection
	QVector<ModeFeature> vectorModeFeature;
	QVector<StateFeature> vectorStateFeature;

	for(auto& mode : modes) {
		std::stringstream modeData(mode);
		std::string cmd, name, values;
		if(getline(modeData, cmd, '|'))
			if(getline(modeData, name, '|'))
				getline(modeData, values, '|');
		vectorModeFeature.append(ModeFeature{cmd.c_str(), name.c_str(), values.c_str(), false});
	}
	// for(auto& state : states)
	// 	vectorStateFeature.append(StateFeature{state.c_str(), false});

	// create state and mode
	modeTomoView = new ModeTomoView();
	modeTomoView->setModel(vectorModeFeature);
	stateTomoView = new StateTomoView();
	stateTomoView->setModel(vectorStateFeature);

	createConnection();
}

TomoVM::~TomoVM()
{
	master_app->control_comm->close();
	isClosing = true;
	if(comm_thread) {
		comm_thread->join();
		comm_thread.reset();
		comm_thread = nullptr;
	}

	delete modeTomoView;
	delete stateTomoView;
}

QVector<TrackVM*> TomoVM::listTrackVM()
{
	return _listTrackVM;
}

QObject* TomoVM::trackListAt(int index) const
{
	if(index < 0 || index >= _listTrackVM.size())
		return nullptr;
	return _listTrackVM[index];
}

void TomoVM::createConnection()
{
	connect(modeTomoView, SIGNAL(currentListModeChanged(std::string)), this, SLOT(modeIsChanged(std::string)));
	connect(stateTomoView, SIGNAL(currentListStateChanged(std::string)), this, SLOT(stateIsChanged(std::string)));
	connect(master_app, SIGNAL(broadcastPressButtonSignal(QString, QString, bool)), this,
			SLOT(broadcastPressButtonSlot(QString, QString, bool)));
}

void TomoVM::callRvizClass()
{
	Q_EMIT enableAllLayouts(false);
	if(_isRvizReadyToCall) {
		if(!_isRvizCalled) {
			try {
				rviz_app = std::make_shared<rviz::VisualizerApp>(master_app->config, parent());
				rviz_app.get()->setApp(m_qapp);

				if(rviz_app.get()->init(m_argc, m_argv))
					UI_WARN("[TomoVM::callRvizClass] RViz initialized");
				else
					UI_ERROR("[TomoVM::callRvizClass] Failed to initialize RViz");

				_isRvizCalled = true;
			}
			catch(const std::exception& e) {
				UI_ERROR("[TomoVM::callRvizClass] RViz Exception: %s", e.what());
			}
		}
		else
			rviz_app->setRvizVisible(true);
	}
	if(rviz_app) {
		connect(rviz_app.get(), SIGNAL(initRvizWindow()), this, SLOT(initRvizWindow()));
		connect(this, SIGNAL(sendRVizMetric(bool, int, int, int, int, bool)), rviz_app.get(),
				SLOT(setRVizMetric(bool, int, int, int, int, bool)));
		connect(this, SIGNAL(sendRvizVisible(bool)), rviz_app.get(), SLOT(setRvizVisible(bool)));
		Q_EMIT requestConnectRviz();
		Q_EMIT rviz_app.get()->initRvizWindow();
	}
	Q_EMIT enableAllLayouts(true);
}

void TomoVM::hiddenRvizWindow()
{
	Q_EMIT enableAllLayouts(false);
	if(_isRvizReadyToCall && rviz_app) {
		disconnect(rviz_app.get(), SIGNAL(initRvizWindow()), this, SLOT(initRvizWindow()));
		disconnect(this, SIGNAL(sendRVizMetric(bool, int, int, int, int, bool)), rviz_app.get(),
				   SLOT(setRVizMetric(bool, int, int, int, int, bool)));
		disconnect(this, SIGNAL(sendRvizVisible(bool)), rviz_app.get(), SLOT(setRvizVisible(bool)));
		Q_EMIT requestDisconnectRviz();
		rviz_app->setRvizVisible(false);
		_isRvizCalled = true;
		UI_WARN("[TomoVM::hiddenRvizWindow] RViz hidden");
	}
	Q_EMIT enableAllLayouts(true);
}

void TomoVM::initRvizWindow()
{
	Q_EMIT initRvizStartState();
}

void TomoVM::updateRVizMetric(bool isVisible, int x, int y, int width, int height, bool isMaximized)
{
	Q_EMIT sendRVizMetric(isVisible, x, y, width, height, isMaximized);
}

void TomoVM::updateRvizVisible(bool value)
{
	Q_EMIT sendRvizVisible(value && master_app->config.tabIndex == DASH_BOARD);
}

void TomoVM::setAfterUiCreated()
{
	Q_EMIT setTabViewIndex(master_app->config.getCameraFocusFromString(master_app->config.camsFocus.at(0)));
	Q_EMIT setEnableRvizBtn(_isRvizReadyToCall);
	Q_EMIT setStateConnectionTomOControl(true);
}

void TomoVM::setListTrackVM(QVector<TrackVM*>& value)
{
	if(_listTrackVM != value) {
		_listTrackVM = value;
		Q_EMIT listTrackVMChanged();
	}
}

void TomoVM::doubleClickSlots(QString nameDoc)
{
	master_app->double_click_views[nameDoc] = !master_app->double_click_views.value(nameDoc);
	master_app->config.writeFileConfig();
}

void TomoVM::broadcastPressButtonSlot(QString camName, QString docName, bool isClick)
{
	if(camName != "TomoPose")
		return;
	Q_EMIT buttonDocStatusChanged(camName, docName, isClick);
}

void TomoVM::clickToolButton(QString trackName, QString buttonName, bool state)
{
	if(trackName != "TomoPose")
		return;
	if(buttonName != "rviz")
		return;
	// state ? callRvizClass() : hiddenRvizWindow();
	if(state) {
		if(buttonName == "rviz")
			callRvizVisualizer();
	}
	else {
		std::string cmd = std::string("sudo pkill -9 rviz");
		int status		= system(cmd.c_str());
	}
}

void TomoVM::callRvizVisualizer()
{
	std::thread([this]() {
		std::string cmd =
			std::string("sudo bash ") + getenv("HOME") + std::string("/ui_moduleinstall/share/tomo_ui/config/command/run_rviz.sh");
		int status = system(cmd.c_str());
	}).detach();
}

void TomoVM::tabViewChange(int index)
{
	if(index < 0)
		return;
	master_app->config.camsFocus.at(0) = cameraFocusToString[static_cast<CAMERAS_FOCUS>(index + 00)];
	master_app->config.writeFileConfig();
}

void TomoVM::stateIsChanged(std::string stateId)
{
	if(!master_app->control_comm) {
		UI_ERROR("TomoVM.stateChanged: Comm node is invalid");
		return;
	}
	UI_INFO("[TomoVM::stateIsChanged]  %s  master_app->control_comm->sendMessage: e + stateId ", stateId.c_str());
	master_app->control_comm->sendMessage("e" + stateId);
}

void TomoVM::modeIsChanged(std::string config)
{
	if(!master_app->control_comm) {
		UI_ERROR("TomoVM.modeChanged: Comm node is invalid");
		return;
	}
	UI_INFO("[TomoVM::modeIsChanged]  %s ", config.c_str());

	// QML script is wrongly sending irrelevant data; Fixing it here temporarily; Udupa; 17Aug'23
	std ::vector<std::string> splits, splits2;
	boost::split(splits, config, boost::is_any_of("|"));
	config		  = "";
	int index	  = 0;
	int modeCount = modes.size();
	for(auto& split : splits) {
		if(!split.empty()) {
			if(index < modeCount) {
				char mode = modes[index][0];
				if(mode == 'v' || mode == 'a' || mode == 'p') {
					std::string str = modes[index].substr(modes[index].rfind('|') + 1);
					boost::split(splits2, str, boost::is_any_of(";"));
					split = splits2[std::stoi(split)];
				}
				config += mode + split + "|";
				index++;
			}
		}
	}
	UI_INFO("[TomoVM::modeIsChanged]  %s, master_app->control_comm->sendMessage: config ", config.c_str());
	master_app->control_comm->sendMessage(config);
}

void TomoVM::closeRvizApp()
{
	if(rviz_app.get())
		rviz_app.get()->closeApp();
}

void TomoVM::setModeTomo(std::string cmd, int value)
{
	modeTomoView->setMode(cmd, value);
}

void TomoVM::tabIndexChange(int index)
{
	return;
}
void TomoVM::setRvizReadyToCall(bool value)
{
	_isRvizReadyToCall = value;
	Q_EMIT setEnableRvizBtn(_isRvizReadyToCall);
}
void TomoVM::setEnbaleModesStates(bool value)
{
	return;
}

int TomoVM::setProductionControl(int type, std::string str)
{
	if(!str.rfind("lot_starting_", 0) || !str.rfind("lot_resuming_", 0)) {
		if(type < 0) {
			int pack = stringToInt(str.substr(str.rfind("_") + 1));
			if(pack == INT_MAX)
				pack = 30;
			production_vm->setCurrentModel(pack);
			production_vm->setIsStartNewLot(!str.rfind("lot_starting_", 0));
			production_vm->setQuantityValue(0);
			production_vm->setActualQuantity(0);
			production_vm->setTotalBlowouts(0);
			production_vm->setTotalAudit(0);
			production_vm->showStartLot();
		}
	}
	else if(str == "lot_start") {
		if(type < 0)
			production_vm->setEndLotEnabled(true);
	}
	else if(str == "lot_ending") {
		if(type < 0) {
			production_vm->endLotPopup();
			production_vm->setEndLotEnabled(false);
		}
	}
	else if(str == "lot_end") {
		if(type < 0)
			production_vm->setEnabledForRunLot(true);
	}
	else if(str == "lot_end_confirm") {
		if(type < 0)
			production_vm->showEndLot();
	}
	else if(str == "lot_abort") {
		if(type < 0)
			production_vm->showAbortLot();
	}
	else if(str == "lot_init") {
		production_vm->setInitialize(type < 0);
	}
	else {
		if(!str.rfind("estop", 0)) {
			master_app->stopProgressBar();
			production_vm->setEnabledForRunLot(true);
			Q_EMIT production_vm->emergencyEvent(type == -3, convertToQString(str));
			production_vm->setEndLotEnabled(false);
		}
		else if(!str.rfind("guard", 0) || !str.rfind("om_int_t", 0) || !str.rfind("im_int", 0) || !str.rfind("cm_door", 0)) {
			Q_EMIT production_vm->emergencyEvent(type < 0, convertToQString(str));
		}
		else if(str.find("Servo") != string::npos) {
			master_app->station_vm->servo_error = true;
			if(str.find("Delta Servo") != string::npos)
				master_app->setSystemReady(false);
		}
		else if(str == "wago") {
			if(type < 0)
				master_app->setSystemReady(false);
		}
		else if(!str.rfind("config", 0) || !str.rfind("ethercat", 0) || !str.rfind("estop", 0)) {
			if(type < 0) {
				master_app->stopProgressBar();
				if(!str.rfind("config", 0))
					master_app->production_vm->setFileValid(false);
			}
		}

		return -1;
	}

	return 0;
}

void TomoVM::setSuperUserAct(bool value)
{
	_isSuperUserAct = value;
}

void TomoVM::testAlarm(int type, QString code)
{
	master_app->modalDialog.showErrorMessage(type, convertToStdString(code));
	setProductionControl(type, convertToStdString(code));
}