#include "station_vm.h"
#include <ros/ros.h>

using namespace tomo_peripherals;
using namespace std;
using namespace std::this_thread;  // sleep_for, sleep_until

// StationVM::StationVM(tClientContainer* client, QObject* parent) : master_app->client(client), QObject{parent}
StationVM::StationVM(QObject* parent, MasterApp* masterApp) : QObject{parent}, master_app(masterApp)
{
	if(!master_app->client)
		UI_ERROR("[StationVM::StationVM] Client container is null pointer !!!");
	_stationThreadAxis = nullptr;
	_stateThread	   = false;
	_stationThreadcv.notify_all();
}

StationVM::~StationVM()
{
	if(!_stationThreadAxis)
		close();
}

void StationVM::close()
{
	_closeStationThread = true;
	_stateThread		= true;
	_stationThreadcv.notify_all();
	if(_stationThreadAxis) {
		_stationThreadAxis->join();
		_stationThreadAxis.reset();
		_stationThreadAxis = nullptr;
	}
}

void StationVM::changeReceivePosValMode(bool value)
{
	if(!value) {
		close();
		return;
	}
	_closeStationThread = !value;
	if(!_stationThreadAxis)
		_stationThreadAxis = std::make_shared<std::thread>(&StationVM::stationThreadAxis, this);
}

void StationVM::startReceivePosValue(int currentStation)
{
	changeThreadState(currentStation == BSTIM_STATION || currentStation == TSTIM_STATION || currentStation == AFOLD_STATION ||
					  currentStation == ITPNP_STATION || currentStation == OTPNP_STATION);
	_currentStation = currentStation;
	if(currentStation == BSTIM_STATION)
		setCurrentAxis(BSTM_AXIS);
	else if(currentStation == TSTIM_STATION)
		setCurrentAxis(TSTM_AXIS);
	else if(currentStation == AFOLD_STATION)
		setCurrentAxis((MotorAxis) _afCurrentAxis);
	else if(currentStation == ITPNP_STATION)
		setCurrentAxis((MotorAxis) _itCurrentAxis);
	else if(currentStation == OTPNP_STATION)
		setCurrentAxis((MotorAxis) _otCurrentAxis);
	else
		return;
}

void StationVM::changeThreadState(bool value)
{
	if(_stateThread != value) {
		_stateThread = value;
		_stationThreadcv.notify_all();
	}
}

int StationVM::stationThreadAxis()
{
	UI_WARN("StationThread: Waiting for server...");
	master_app->client->waitForServer();
	UI_WARN("StationThread: Running");
	int currentPos, currentState;  // currentState: 0:AtRest; 1:Homing; 2:Moving
	while(!_closeStationThread && master_app->powerUpState()) {
		std::unique_lock<std::mutex> lock(_stationThreadMutex);
		_stationThreadcv.wait(lock, [this] { return _stateThread; });

		if(_closeStationThread)
			return -1;
		if(!master_app->client) {
			UI_ERROR("[StationVM::stationThread] Client not init!!!");
			return -1;
		}

		if(servo_error || !master_app->systemReady()) {
			UI_ERROR("[StationVM::stationThread]: Lost communication");
			return -1;
		}
		switch(_currentStation) {
		case BSTIM_STATION: {
			master_app->client->dmGetCurState(BSTM_AXIS, currentPos, currentState);
			setBsCurrentPosition(currentPos);
			setReadyToMove(BSTM_AXIS, currentState, false);
		} break;
		case TSTIM_STATION: {
			master_app->client->dmGetCurState(TSTM_AXIS, currentPos, currentState);
			setTsCurrentPosition(currentPos);
			setReadyToMove(TSTM_AXIS, currentState, false);
		} break;
		case ITPNP_STATION: {
			bool itMoveCheck = true;
			master_app->client->dmGetCurState(INPUT_PNP_X_AXIS, currentPos, currentState);
			setItXAxisPosition(currentPos);
			setReadyToMove(INPUT_PNP_X_AXIS, currentState, false);
			master_app->client->dmGetCurState(INPUT_PNP_Y_AXIS, currentPos, currentState);
			setItYAxisPosition(currentPos);
			setReadyToMove(INPUT_PNP_Y_AXIS, currentState, false);
		} break;
		case AFOLD_STATION: {
			bool afMoveCheck = true;
			master_app->client->dmGetCurState(AFOLD_X_AXIS, currentPos, currentState);
			setAfXAxisPosition(currentPos);
			setReadyToMove(AFOLD_X_AXIS, currentState, false);
			master_app->client->dmGetCurState(AFOLD_Y_AXIS, currentPos, currentState);
			setAfYAxisPosition(currentPos);
			setReadyToMove(AFOLD_Y_AXIS, currentState, false);
		} break;
		case OTPNP_STATION: {
			master_app->client->dmGetCurState(OUTPUT_PNP_X_AXIS, currentPos, currentState);
			setOtXAxisPosition(currentPos);
			if(waitHomeAllDone)
				setReadyToMove(OUTPUT_PNP_X_AXIS, currentState, false);
			master_app->client->dmGetCurState(OUTPUT_PNP_Y_AXIS, currentPos, currentState);
			setOtYAxisPosition(currentPos);
			if(waitHomeAllDone)
				setReadyToMove(OUTPUT_PNP_Y_AXIS, currentState, false);
			master_app->client->dmGetCurState(OUTPUT_PNP_Z_AXIS, currentPos, currentState);
			setOtZAxisPosition(currentPos);
			if(waitHomeAllDone)
				setReadyToMove(OUTPUT_PNP_Z_AXIS, currentState, false);
		} break;
		}

		lock.unlock();
		usleep(500000);
	}

	return 0;
}
void StationVM::goToIoTab(int ioTabIndex)
{
	Q_EMIT callIOTab(IO);
	Q_EMIT setTabIndexIO(ioTabIndex);
}

void StationVM::homeAxis(int axis)
{
	if(servo_error || !master_app->systemReady()) {
		UI_ERROR("[StationVM::homeAxis]: Lost communication");
		master_app->modalDialog.getModalDialog(false, "system_lost");
		return;
	}
	if(!master_app->client) {
		UI_ERROR("[StationVM::homeAxis] Can not call client, please check it!!!");
		return;
	}
	std::thread([this, axis]() {
		UI_WARN("[StationVM::homeAxis] Call client dmHomeAxis: %d", axis);
		master_app->client->dmHomeAxis(CMD_SEND, axis, true);
		waitForState = false;
		setReadyToMove(axis, 1, true);
		sleep_for(500ms);
		waitForState = true;
	}).detach();
}

void StationVM::moveAxis(int axis, int pos, int isAbs, int rpm)
{
	if(servo_error || !master_app->systemReady()) {
		UI_ERROR("[StationVM::moveAxis]: Lost communication");
		master_app->modalDialog.getModalDialog(false, "system_lost");
		return;
	}
	if(!master_app->client) {
		UI_ERROR("[StationVM::moveAxis] Can not call client, please check it!!!");
		return;
	}

	std::thread([this, axis, pos, isAbs, rpm]() {
		UI_WARN("[StationVM::moveAxis] Call client dmMoveAxis: %d, position: %d, isAbs: %d", axis, pos, isAbs);
		master_app->client->dmMoveAxis(CMD_SEND, axis, pos, isAbs, rpm);
		waitForState = false;
		setReadyToMove(axis, 2, true);
		sleep_for(500ms);
		waitForState = true;
	}).detach();
}

void StationVM::stopAxis(int axis)
{
	if(servo_error || !master_app->systemReady()) {
		UI_ERROR("[StationVM::moveAxis]: Lost communication");
		master_app->modalDialog.getModalDialog(false, "system_lost");
		return;
	}
	if(!master_app->client) {
		UI_ERROR("[StationVM::stopAxis] Can not call client, please check it!!!");
		return;
	}
	std::thread([this, axis]() {
		UI_WARN("[StationVM::homeAxis] Call client dmStopAxis: %d", axis);
		master_app->client->dmStopAxis(CMD_SEND, axis);
	}).detach();
}

void StationVM::setReadyToMove(int currentAxis, int state, bool bypass)
{
	if(waitForState || bypass) {
		// UI_INFO("[StationVM::setReadyToMove] Current Axis: %s , state: %s", AxisToName[currentAxis].c_str(), state == 0 ? "AtRest" :
		// (state == 1 ? "Homing" : (state == 2 ? "Moving" : "CustomState")));
		_tempMoveVector[currentAxis] = state;
		setMoveStateVector(_tempMoveVector);
	}
}

QVector<int> StationVM::moveStateVector()
{
	return _moveStateVector;
}

void StationVM::setMoveStateVector(QVector<int> value)
{
	if(_moveStateVector != value) {
		for(int i = 1; i < value.size(); i++) {
			if(_moveStateVector[i] != value[i]) {
				int state = value[i];
				UI_INFO("[StationVM::setReadyToMove] Current Axis: %s , state: %s", AxisToName[i].c_str(),
						state == 0 ? "AtRest" : (state == 1 ? "Homing" : (state == 2 ? "Moving" : "CustomState")));
			}
		}
		_moveStateVector = value;
		moveStateVectorChanged(_moveStateVector);
	}
}
void StationVM::setCurrentLimit()
{
	if(!master_app->client) {
		UI_ERROR("[StationVM::setCurrentLimit] Can not call client, please check it!!!");
		return;
	}
	if(_currentAxis < AFOLD_X_AXIS || _currentAxis > OUTPUT_PNP_Z_AXIS) {
		UI_ERROR("[StationVM::setCurrentLimit] Invalid Axis, please check it!!!");
		return;
	}

	std::thread([this]() {
		int minLimit, maxLimit;
		if(!master_app->client->dmGetAxisLimits(_currentAxis, minLimit, maxLimit)) {
			return;
		}
		setCurrentMinLimitPosition(minLimit);
		setCurrentMaxLimitPosition(maxLimit);
		UI_INFO("[StationVM::setCurrentLimit] Current Axis: %d, Min Limit Now: %d, Max Limit Now: %d ", _currentAxis, minLimit, maxLimit);
	}).detach();
}

void StationVM::setSpeedLimit()
{
	int minLimit, maxLimit;
	minLimit = motorAxisLimitSpeed[_currentAxis].second;
	maxLimit = motorAxisLimitSpeed[_currentAxis].first;
	setCurrentMinLimitSpeed(minLimit);
	setCurrentMaxLimitSpeed(maxLimit);
	UI_INFO("[StationVM::setSpeedLimit] Current Axis: %d, Min Speed Now: %d, Max Speed Now: %d ", _currentAxis, minLimit, maxLimit);
}

int StationVM::currentAxis()
{
	return _currentAxis;
}

void StationVM::setCurrentAxis(int value)
{
	if(value == BSTM_AXIS || value == TSTM_AXIS) {
		if(servo_error || !master_app->systemReady()) {
			UI_ERROR("[StationVM::setCurrentAxis]: Lost communication");
			master_app->modalDialog.getModalDialog(false, "system_lost");
			return;
		}
	}
	_currentAxis = value;
	setCurrentLimit();
	setSpeedLimit();
	Q_EMIT getLimitSlider();
	Q_EMIT currentAxisChanged();
}

int StationVM::currentMinLimitPosition()
{
	return _currentMinLimitPosition;
}

void StationVM::setCurrentMinLimitPosition(int value)
{
	if(value != _currentMinLimitPosition) {
		_currentMinLimitPosition = value;
		Q_EMIT currentMinLimitPositionChanged();
	}
}

int StationVM::currentMaxLimitPosition()
{
	return _currentMaxLimitPosition;
}

void StationVM::setCurrentMaxLimitPosition(int value)
{
	if(value != _currentMaxLimitPosition) {
		_currentMaxLimitPosition = value;
		Q_EMIT currentMaxLimitPositionChanged();
	}
}

int StationVM::currentMinLimitSpeed()
{
	return _currentMinLimitSpeed;
}

void StationVM::setCurrentMinLimitSpeed(int value)
{
	if(value != _currentMinLimitSpeed) {
		_currentMinLimitSpeed = value;
		Q_EMIT currentMinLimitSpeedChanged();
	}
}

int StationVM::currentMaxLimitSpeed()
{
	return _currentMaxLimitSpeed;
}

void StationVM::setCurrentMaxLimitSpeed(int value)
{
	if(value != _currentMaxLimitSpeed) {
		_currentMaxLimitSpeed = value;
		Q_EMIT currentMaxLimitSpeedChanged();
	}
}

void StationVM::getCurPos(int value)
{
	_currentPosition = value;
}

void StationVM::requestParmButtonState(int currentStation){
	int moduleId = StationToModule[currentStation];	
	// Send this message to check if the module has any parameters; Call this message when opening station page, to enable/disable
	// Parameters button
	master_app->control_comm->sendMessage(CMD_UI + std::to_string(CMD_UI_PARM) + "^" + std::to_string(moduleId) + "^0");
}
bool StationVM::enabledParmButton(){
	return _enabledParmButton;
}
void StationVM::setEnabledParmButton(bool value){
	_enabledParmButton = value;
	enabledParmButtonChanged();
}

void StationVM::requestParmList(int currentStation){
	UI_INFO("[StationVM::requestParmList] Current station: %s", StationToName[currentStation].c_str());
	/////////////////////////// Udupa; 04May'24
	int moduleId = StationToModule[currentStation];	// Convert currentStation to actual moduleId (refer to ModuleId enum in def.server.h)
									// Use GRIPPER_MODULE for TomO station
									// Use TRAY_MODULE for TSTIM or BSTIM station
									// Use CARTON_TRANSFER_MODULE for PIM or conveyor stations
	// Send this message to receive module's parameter list; Call this message when Parameters button is clicked
	master_app->control_comm->sendMessage(CMD_UI + std::to_string(CMD_UI_PARM) + "^" + std::to_string(moduleId) + "^1");
}

QStringList StationVM::parametersList()
{
	return _parametersList;
}

void StationVM::setParametersList(QStringList value)
{
	UI_INFO("[StationVM::setParametersList] Size: %d", value.size());
	_parametersList = value;
	parametersListChanged();
}

int StationVM::getPosition(QString axis)
{
	UI_INFO("[StationVM::getPosition]");
	return 0;
}

bool StationVM::getRequireStart(QString nameParameter)
{
	UI_INFO("[StationVM::getRequireStart]");
	return true;
}

void StationVM::setPosition(QString axis, int pos)
{
	UI_INFO("[StationVM::setPosition]");
}
void StationVM::setParameters(int currentStation, QString parmValue)
{
	UI_INFO("[StationVM::setParameters] Current station: %s", StationToName[currentStation].c_str());
	// Udupa; 04May'04
	/// Change setParameters to set ALL parameters for the active module (currently it seems, this function sets only one parameter at a time)
	/// Should pass station no. as parameter
	int moduleId = StationToModule[currentStation];	// Convert currentStation to actual moduleId (refer to ModuleId enum in def.server.h)
									// Use GRIPPER_MODULE for TomO station
									// Use TRAY_MODULE for TSTIM or BSTIM station
									// Use CARTON_TRANSFER_MODULE for PIM or conveyor stations

	std::string parmMessage = CMD_UI + std::to_string(CMD_UI_PARM) + "^" + std::to_string(moduleId) + "^2";

	// Udupa; 04May'04
	// for each parameter in the module
	//      add to the parmMessage string like this:
	//      parmMessage += "^" + std::to_string(newVal[i]);
	parmMessage += convertToStdString(parmValue);
	master_app->control_comm->sendMessage(parmMessage);

	UI_INFO("[StationVM::setParameters] Publish message: %s", parmMessage.c_str());
}
void StationVM::setRequireStart(QString nameParameter)
{
	UI_INFO("[StationVM::setRequireStart]");
}
// 'bs' BSTIM
void StationVM::bsHomeClicked()
{
	UI_INFO("StationVM::bsHomeClicked");
	if(servo_error || !master_app->systemReady()) {
		UI_ERROR("[StationVM::bsHomeClicked]: Lost communication");
		master_app->modalDialog.getModalDialog(false, "system_lost");
		return;
	}
	setCurrentAxis(BSTM_AXIS);
	master_app->modalDialog.getModalDialog(true, "bs_home");
}

void StationVM::bsStopClicked()
{
	UI_INFO("StationVM::bsStopClicked");
	stopAxis(BSTM_AXIS);
}

void StationVM::bsCheckTrayClicked()
{
	UI_INFO("StationVM::bsCheckTrayClicked");
	return;
}

void StationVM::bsFeedTrayClicked()
{
	UI_INFO("StationVM::bsFeedTrayClicked");
	return;
}

void StationVM::bsMoveClicked()
{
	UI_INFO("StationVM::bsMoveClicked");
	if(servo_error || !master_app->systemReady()) {
		UI_ERROR("[StationVM::bsHomeClicked]: Lost communication");
		master_app->modalDialog.getModalDialog(false, "system_lost");
		return;
	}
	setCurrentAxis(BSTM_AXIS);
	master_app->modalDialog.getModalDialog(true, "bs_move");
}

void StationVM::bsBackwardClicked()
{
	UI_INFO("StationVM::bsBackwardClicked");
	if(servo_error || !master_app->systemReady()) {
		UI_ERROR("[StationVM::bsBackwardClicked]: Lost communication");
		master_app->modalDialog.getModalDialog(false, "system_lost");
		return;
	}
	setCurrentAxis(BSTM_AXIS);
	master_app->modalDialog.getModalDialog(true, "bs_back");
}

void StationVM::bsForwardClicked()
{
	UI_INFO("StationVM::bsForwardClicked");
	if(servo_error || !master_app->systemReady()) {
		UI_ERROR("[StationVM::bsForwardClicked]: Lost communication");
		master_app->modalDialog.getModalDialog(false, "system_lost");
		return;
	}
	setCurrentAxis(BSTM_AXIS);
	master_app->modalDialog.getModalDialog(true, "bs_forw");
}

int StationVM::bsSpeedSlider()
{
	return _bsSpeedSlider;
}

void StationVM::setBsSpeedSlider(int value)
{
	_bsSpeedSlider = value;
	Q_EMIT bsSpeedSliderChanged(MOTOR_S1, value);
}

int StationVM::bsPosition()
{
	return _bsPosition;
}

void StationVM::setBsPosition(int value)
{
	if(value == _bsPosition)
		return;
	_bsPosition = value;
	setCurrentAxis(BSTM_AXIS);
	Q_EMIT bsPositionChanged();
}

int StationVM::bsDistance()
{
	return _bsDistance;
}

void StationVM::setBsDistance(int value)
{
	if(value == _bsDistance)
		return;
	_bsDistance = value;
	setCurrentAxis(BSTM_AXIS);
	Q_EMIT bsDistanceChanged();
}

int StationVM::bsCurrentPosition()
{
	return _bsCurrentPosition;
}

void StationVM::setBsCurrentPosition(int value)
{
	if(value != _bsCurrentPosition) {
		_bsCurrentPosition = value;
		setCurrentLimit();
		Q_EMIT bsCurrentPositionChanged();
	}
}

int StationVM::bsCurrentSpeed()
{
	return _bsCurrentSpeed;
}

void StationVM::setBsCurrentSpeed(int value)
{
	if(value != _bsCurrentSpeed) {
		_bsCurrentSpeed = value;
		setCurrentLimit();
		Q_EMIT bsCurrentSpeedChanged();
	}
}

//'ts' TSTIM
void StationVM::tsHomeClicked()
{
	UI_INFO("StationVM::tsHomeClicked");
	if(servo_error || !master_app->systemReady()) {
		UI_ERROR("[StationVM::tsHomeClicked]: Lost communication");
		master_app->modalDialog.getModalDialog(false, "system_lost");
		return;
	}
	setCurrentAxis(TSTM_AXIS);
	master_app->modalDialog.getModalDialog(true, "ts_home");
}

void StationVM::tsStopClicked()
{
	UI_INFO("StationVM::tsStopClicked");
	stopAxis(TSTM_AXIS);
}

void StationVM::tsCheckTrayClicked()
{
	UI_INFO("StationVM::tsCheckTrayClicked");
}

void StationVM::tsFeedTrayClicked()
{
	UI_INFO("StationVM::tsFeedTrayClicked");
}

void StationVM::tsMoveClicked()
{
	UI_INFO("StationVM::tsMoveClicked");
	if(servo_error || !master_app->systemReady()) {
		UI_ERROR("[StationVM::tsMoveClicked]: Lost communication");
		master_app->modalDialog.getModalDialog(false, "system_lost");
		return;
	}
	setCurrentAxis(TSTM_AXIS);
	master_app->modalDialog.getModalDialog(true, "ts_move");
}

void StationVM::tsBackwardClicked()
{
	UI_INFO("StationVM::tsBackwardClicked");
	if(servo_error || !master_app->systemReady()) {
		UI_ERROR("[StationVM::tsBackwardClicked]: Lost communication");
		master_app->modalDialog.getModalDialog(false, "system_lost");
		return;
	}
	setCurrentAxis(TSTM_AXIS);
	master_app->modalDialog.getModalDialog(true, "ts_back");
}

void StationVM::tsForwardClicked()
{
	UI_INFO("StationVM::tsForwardClicked");
	if(servo_error || !master_app->systemReady()) {
		UI_ERROR("[StationVM::tsForwardClicked]: Lost communication");
		master_app->modalDialog.getModalDialog(false, "system_lost");
		return;
	}
	setCurrentAxis(TSTM_AXIS);
	master_app->modalDialog.getModalDialog(true, "ts_forw");
}

int StationVM::tsSpeedSlider()
{
	return _tsSpeedSlider;
}

void StationVM::setTsSpeedSlider(int value)
{
	_tsSpeedSlider = value;
	Q_EMIT tsSpeedSliderChanged(MOTOR_S2, value);
}

int StationVM::tsPosition()
{
	return _tsPosition;
}

void StationVM::setTsPosition(int value)
{
	if(value != _tsPosition) {
		_tsPosition = value;
		setCurrentAxis(TSTM_AXIS);
		Q_EMIT tsPositionChanged();
	}
}

int StationVM::tsDistance()
{
	return _tsDistance;
}

void StationVM::setTsDistance(int value)
{
	if(value != _tsDistance) {
		_tsDistance = value;
		setCurrentAxis(TSTM_AXIS);
		Q_EMIT tsDistanceChanged();
	}
}

int StationVM::tsCurrentPosition()
{
	return _tsCurrentPosition;
}

void StationVM::setTsCurrentPosition(int value)
{
	if(value != _tsCurrentPosition) {
		_tsCurrentPosition = value;
		setCurrentLimit();
		Q_EMIT tsCurrentPositionChanged();
	}
}

int StationVM::tsCurrentSpeed()
{
	return _tsCurrentSpeed;
}

void StationVM::setTsCurrentSpeed(int value)
{
	if(value != _tsCurrentSpeed) {
		_tsCurrentSpeed = value;
		setCurrentLimit();
		Q_EMIT tsCurrentSpeedChanged();
	}
}

// 'it' Input Tray PNP
void StationVM::itHomeAllClicked()
{
	UI_INFO("StationVM::itHomeAllClicked");
	if(servo_error || !master_app->systemReady()) {
		UI_ERROR("[StationVM::itHomeAllClicked]: Lost communication");
		master_app->modalDialog.getModalDialog(false, "system_lost");
		return;
	}
	master_app->modalDialog.getModalDialog(true, "it_home_all");
}

void StationVM::itStopAllClicked()
{
	UI_INFO("StationVM::itStopAllClicked");
	stopAxis(INPUT_PNP_X_AXIS);
	stopAxis(INPUT_PNP_Y_AXIS);
}

void StationVM::itPrepareTray()
{
}

void StationVM::itTransferTray()
{
}

void StationVM::itHomeClicked()
{
	UI_INFO("StationVM::itHomeClicked");
	if(servo_error || !master_app->systemReady()) {
		UI_ERROR("[StationVM::itHomeClicked]: Lost communication");
		master_app->modalDialog.getModalDialog(false, "system_lost");
		return;
	}
	master_app->modalDialog.getModalDialog(true, "it_home");
}

void StationVM::itStopClicked()
{
	UI_INFO("StationVM::itStopClicked");
	stopAxis(_itCurrentAxis);
}

void StationVM::itMoveClicked()
{
	UI_INFO("StationVM::itMoveClicked");
	if(servo_error || !master_app->systemReady()) {
		UI_ERROR("[StationVM::itMoveClicked]: Lost communication");
		master_app->modalDialog.getModalDialog(false, "system_lost");
		return;
	}
	master_app->modalDialog.getModalDialog(true, "it_move");
}

void StationVM::itBackwardClicked()
{
	UI_INFO("StationVM::itBackwardClicked");
	if(servo_error || !master_app->systemReady()) {
		master_app->modalDialog.getModalDialog(false, "system_lost");
		return;
	}
	master_app->modalDialog.getModalDialog(true, "it_back");
}

void StationVM::itForwardClicked()
{
	UI_INFO("StationVM::itForwardClicked");
	if(servo_error || !master_app->systemReady()) {
		UI_ERROR("[StationVM::itForwardClicked]: Lost communication");
		master_app->modalDialog.getModalDialog(false, "system_lost");
		return;
	}
	master_app->modalDialog.getModalDialog(true, "it_forw");
}

int StationVM::itCurrentAxis()
{
	return _itCurrentAxis;
}

void StationVM::setItCurrentAxis(int value)
{
	if(value != _itCurrentAxis) {
		_itCurrentAxis = value;
		setCurrentAxis(value);
		Q_EMIT itCurrentAxisChanged(AXIS_S3, value);
	}
}

int StationVM::itSpeedSliderX()
{
	return _itSpeedSliderX;
}

void StationVM::setItSpeedSliderX(int value)
{
	_itSpeedSliderX = value;
	Q_EMIT itSpeedSliderXChanged(MOTOR_S3X, value);
}

int StationVM::itSpeedSliderY()
{
	return _itSpeedSliderY;
}

void StationVM::setItSpeedSliderY(int value)
{
	_itSpeedSliderY = value;
	Q_EMIT itSpeedSliderYChanged(MOTOR_S3Y, value);
}

int StationVM::itPosition()
{
	return _itPosition;
}

void StationVM::setItPosition(int value)
{
	if(value != _itPosition) {
		_itPosition = value;
		Q_EMIT itPositionChanged();
	}
}

int StationVM::itDistance()
{
	return _itDistance;
}

void StationVM::setItDistance(int value)
{
	if(value != _itDistance) {
		_itDistance = value;
		Q_EMIT itDistanceChanged();
	}
}

int StationVM::itCurrentPosition()
{
	return _itCurrentPosition;
}

void StationVM::setItCurrentPosition(int value)
{
	if(value != _itCurrentPosition) {
		_itCurrentPosition = value;
		Q_EMIT itCurrentPositionChanged();
	}
}

int StationVM::itCurrentSpeed()
{
	return _itCurrentSpeed;
}

void StationVM::setItCurrentSpeed(int value)
{
	if(value != _itCurrentSpeed) {
		_itCurrentSpeed = value;
		setCurrentAxis(value);
		Q_EMIT itCurrentSpeedChanged();
	}
}

int StationVM::itXAxisPosition()
{
	return _itXAxisPosition;
}

void StationVM::setItXAxisPosition(int value)
{
	if(value != _itXAxisPosition) {
		_itXAxisPosition = value;
		Q_EMIT itXAxisPositionChanged();
	}
}

int StationVM::itXAxisSpeed()
{
	return _itXAxisSpeed;
}

void StationVM::setItXAxisSpeed(int value)
{
	if(value != _itXAxisSpeed) {
		_itXAxisSpeed = value;
		Q_EMIT itXAxisPositionChanged();
	}
}

int StationVM::itYAxisPosition()
{
	return _itYAxisPosition;
}

void StationVM::setItYAxisPosition(int value)
{
	if(value != _itYAxisPosition) {
		_itYAxisPosition = value;
		Q_EMIT itYAxisPositionChanged();
	}
}

int StationVM::itYAxisSpeed()
{
	return _itYAxisSpeed;
}

void StationVM::setItYAxisSpeed(int value)
{
	if(value != _itYAxisSpeed) {
		_itYAxisSpeed = value;
		Q_EMIT itYAxisSpeedChanged();
	}
}

//'af' AFolding
void StationVM::afHomeAllClicked()
{
	UI_INFO("StationVM::afHomeAllClicked");
	homeAxis(AFOLD_X_AXIS);
	homeAxis(AFOLD_Y_AXIS);
}

void StationVM::afStopAllClicked()
{
	UI_INFO("StationVM::afStopAllClicked");
	stopAxis(AFOLD_X_AXIS);
	stopAxis(AFOLD_Y_AXIS);
}

void StationVM::afPrepareClicked()
{
	UI_INFO("StationVM::afPrepareClicked");
}

void StationVM::afFoldClicked()
{
	UI_INFO("StationVM::afFoldClicked");
}

void StationVM::afFoldFinishClicked()
{
	UI_INFO("StationVM::afFoldFinishClicked");
}

void StationVM::afHomeClicked()
{
	UI_INFO("StationVM::afHomeClicked");
	if(servo_error || !master_app->systemReady()) {
		UI_ERROR("[StationVM::afHomeClicked]: Lost communication");
		master_app->modalDialog.getModalDialog(false, "system_lost");
		return;
	}
	homeAxis(_afCurrentAxis);
}

void StationVM::afStopClicked()
{
	UI_INFO("StationVM::afStopClicked");
	stopAxis(_afCurrentAxis);
}

void StationVM::afMoveClicked()
{
	UI_INFO("StationVM::afMoveClicked");
	if(servo_error || !master_app->systemReady()) {
		UI_ERROR("[StationVM::afMoveClicked]: Lost communication");
		master_app->modalDialog.getModalDialog(false, "system_lost");
		return;
	}
	switch(_afCurrentAxis) {
	case 1:
		moveAxis(_afCurrentAxis, _afPosition, 1, _afSpeedSliderX);
		break;
	case 2:
		moveAxis(_afCurrentAxis, _afPosition, 1, _afSpeedSliderY);
		break;
	default:
		break;
	}
}

void StationVM::afBackwardClicked()
{
	UI_INFO("StationVM::afBackwardClicked");
	if(servo_error || !master_app->systemReady()) {
		UI_ERROR("[StationVM::afBackwardClicked]: Lost communication");
		master_app->modalDialog.getModalDialog(false, "system_lost");
		return;
	}
	switch(_afCurrentAxis) {
	case 1:
		moveAxis(_afCurrentAxis, _afDistance * -1, 0, _afSpeedSliderX);
		break;
	case 2:
		moveAxis(_afCurrentAxis, _afDistance * -1, 0, _afSpeedSliderY);
		break;
	default:
		break;
	}
}

void StationVM::afForwardClicked()
{
	UI_INFO("StationVM::afForwardClicked");
	if(servo_error || !master_app->systemReady()) {
		UI_ERROR("[StationVM::afForwardClicked]: Lost communication");
		master_app->modalDialog.getModalDialog(false, "system_lost");
		return;
	}
	switch(_afCurrentAxis) {
	case 1:
		moveAxis(_afCurrentAxis, _afDistance, 0, _afSpeedSliderX);
		break;
	case 2:
		moveAxis(_afCurrentAxis, _afDistance, 0, _afSpeedSliderY);
		break;
	default:
		break;
	}
}

int StationVM::afCurrentAxis()
{
	return _afCurrentAxis;
}

void StationVM::setAfCurrentAxis(int value)
{
	if(value != _afCurrentAxis) {
		_afCurrentAxis = value;
		setCurrentAxis(value);
		Q_EMIT afCurrentAxisChanged(AXIS_S4, value);
	}
}

int StationVM::afTrayType()
{
	return _afTrayType;
}

void StationVM::setAfTrayType(int value)
{
	if(value != _afTrayType) {
		_afTrayType = value;
		Q_EMIT afTrayTypeChanged();
	}
}

int StationVM::afSpeedSliderX()
{
	return _afSpeedSliderX;
}

void StationVM::setAfSpeedSliderX(int value)
{
	_afSpeedSliderX = value;
	Q_EMIT afSpeedSliderXChanged(MOTOR_S4X, value);
}
int StationVM::afSpeedSliderY()
{
	return _afSpeedSliderY;
}

void StationVM::setAfSpeedSliderY(int value)
{
	_afSpeedSliderY = value;
	Q_EMIT afSpeedSliderYChanged(MOTOR_S4Y, value);
}

int StationVM::afPosition()
{
	return _afPosition;
}

void StationVM::setAfPosition(int value)
{
	if(value != _afPosition) {
		_afPosition = value;
		Q_EMIT afPositionChanged();
	}
}

int StationVM::afDistance()
{
	return _afDistance;
}

void StationVM::setAfDistance(int value)
{
	if(value != _afDistance) {
		_afDistance = value;
		Q_EMIT afDistanceChanged();
	}
}

int StationVM::afXAxisPosition()
{
	return _afXAxisPosition;
}

void StationVM::setAfXAxisPosition(int value)
{
	if(value != _afXAxisPosition) {
		_afXAxisPosition = value;
		Q_EMIT afXAxisPositionChanged();
	}
}

int StationVM::afXAxisSpeed()
{
	return _afXAxisSpeed;
}

void StationVM::setAfXAxisSpeed(int value)
{
	if(value != _afXAxisSpeed) {
		_afXAxisSpeed = value;
		Q_EMIT afXAxisSpeedChanged();
	}
}

int StationVM::afYAxisPosition()
{
	return _afYAxisPosition;
}

void StationVM::setAfYAxisPosition(int value)
{
	if(value != _afYAxisPosition) {
		_afYAxisPosition = value;
		Q_EMIT afYAxisPositionChanged();
	}
}

int StationVM::afYAxisSpeed()
{
	return _afYAxisSpeed;
}

void StationVM::setAfYAxisSpeed(int value)
{
	if(value != _afYAxisSpeed) {
		_afYAxisSpeed = value;
		Q_EMIT afYAxisSpeedChanged();
	}
}

//'tg' Tomo Gripper
void StationVM::tgGripperAction()
{
}

int StationVM::tgGripperValue()
{
	return _tgGripperValue;
}

void StationVM::setTgGripperValue(int value)
{
	if(value != _tgGripperValue) {
		_tgGripperValue = value;
		Q_EMIT tgGripperValueChanged();
	}
}

void StationVM::tgCheckGripper()
{
}

void StationVM::tgVacuumOn()
{
}

void StationVM::tgCheckVacuum()
{
}

void StationVM::tgVacuumBlow()
{
}

void StationVM::tgPressureSet()
{
}

// 'cl' Carton Loader
void StationVM::clTrayClamp()
{
}

void StationVM::clTrayTilt()
{
}

void StationVM::clTrayTiltAndClamp()
{
}

void StationVM::clTrayPushOutBox()
{
}

//'ct' Carton Transfer
void StationVM::ctCartonPicked()
{
}

void StationVM::ctWaitStation()
{
}

void StationVM::ctGetStation()
{
}

void StationVM::ctWaitFill()
{
}

void StationVM::otHomeAllClicked()
{
	UI_INFO("StationVM::otHomeAllClicked");
	master_app->modalDialog.getModalDialog(true, "ot_home_all");
}

void StationVM::otStopAllClicked()
{
	UI_INFO("StationVM::otStopAllClicked");
	stopAxis(OUTPUT_PNP_X_AXIS);
	stopAxis(OUTPUT_PNP_Y_AXIS);
	stopAxis(OUTPUT_PNP_Z_AXIS);
}

void StationVM::otTransferClicked()
{
	UI_INFO("StationVM::otTransferClicked");
}

void StationVM::otHomeClicked()
{
	UI_INFO("StationVM::otHomeClicked");
	if(servo_error || !master_app->systemReady()) {
		UI_ERROR("[StationVM::otHomeClicked]: Lost communication");
		master_app->modalDialog.getModalDialog(false, "system_lost");
		return;
	}
	master_app->modalDialog.getModalDialog(true, "ot_home");
}

void StationVM::otStopClicked()
{
	UI_INFO("StationVM::otStopClicked");
	stopAxis(_otCurrentAxis);
}

void StationVM::otMoveClicked()
{
	UI_INFO("StationVM::otMoveClicked");
	if(servo_error || !master_app->systemReady()) {
		UI_ERROR("[StationVM::otMoveClicked]: Lost communication");
		master_app->modalDialog.getModalDialog(false, "system_lost");
		return;
	}
	master_app->modalDialog.getModalDialog(true, "ot_move");
}

void StationVM::otBackwardClicked()
{
	UI_INFO("StationVM::otBackwardClicked");
	if(servo_error || !master_app->systemReady()) {
		UI_ERROR("[StationVM::otBackwardClicked]: Lost communication");
		master_app->modalDialog.getModalDialog(false, "system_lost");
		return;
	}
	master_app->modalDialog.getModalDialog(true, "ot_back");
}

void StationVM::otForwardClicked()
{
	UI_INFO("StationVM::otForwardClicked");
	if(servo_error || !master_app->systemReady()) {
		UI_ERROR("[StationVM::otForwardClicked]: Lost communication");
		master_app->modalDialog.getModalDialog(false, "system_lost");
		return;
	}
	master_app->modalDialog.getModalDialog(true, "ot_forw");
}

int StationVM::otCurrentAxis()
{
	return _otCurrentAxis;
}

void StationVM::setOtCurrentAxis(int value)
{
	UI_INFO("StationVM::setOtCurrentAxis");
	if(value != _otCurrentAxis) {
		_otCurrentAxis = value;
		setCurrentAxis(value);
		Q_EMIT otCurrentAxisChanged(AXIS_S9, value);
	}
}

int StationVM::otSpeedSliderX()
{
	return _otSpeedSliderX;
}

void StationVM::setOtSpeedSliderX(int value)
{
	_otSpeedSliderX = value;
	Q_EMIT otSpeedSliderXChanged(MOTOR_S9X, value);
}
int StationVM::otSpeedSliderY()
{
	return _otSpeedSliderY;
}

void StationVM::setOtSpeedSliderY(int value)
{
	_otSpeedSliderY = value;
	Q_EMIT otSpeedSliderYChanged(MOTOR_S9Y, value);
}
int StationVM::otSpeedSliderZ()
{
	return _otSpeedSliderZ;
}

void StationVM::setOtSpeedSliderZ(int value)
{
	_otSpeedSliderZ = value;
	Q_EMIT otSpeedSliderZChanged(MOTOR_S9Z, value);
}

int StationVM::otPosition()
{
	return _otPosition;
}

void StationVM::setOtPosition(int value)
{
	if(value != _otPosition) {
		_otPosition = value;
		Q_EMIT otPositionChanged();
	}
}

int StationVM::otDistance()
{
	return _otDistance;
}

void StationVM::setOtDistance(int value)
{
	if(value != _otDistance) {
		_otDistance = value;
		Q_EMIT otDistanceChanged();
	}
}

int StationVM::otXAxisPosition()
{
	return _otXAxisPosition;
}

void StationVM::setOtXAxisPosition(int value)
{
	if(value != _otXAxisPosition) {
		_otXAxisPosition = value;
		Q_EMIT otXAxisPositionChanged();
	}
}

int StationVM::otXAxisSpeed()
{
	return _otXAxisSpeed;
}

void StationVM::setOtXAxisSpeed(int value)
{
	if(value != _otXAxisSpeed) {
		_otXAxisSpeed = value;
		Q_EMIT otXAxisSpeedChanged();
	}
}

int StationVM::otYAxisPosition()
{
	return _otYAxisPosition;
}

void StationVM::setOtYAxisPosition(int value)
{
	if(value != _otYAxisPosition) {
		_otYAxisPosition = value;
		Q_EMIT otYAxisPositionChanged();
	}
}

int StationVM::otZAxisPosition()
{
	return _otZAxisPosition;
}

void StationVM::setOtZAxisPosition(int value)
{
	if(value != _otZAxisPosition) {
		_otZAxisPosition = value;
		Q_EMIT otZAxisPositionChanged();
	}
}

int StationVM::otYAxisSpeed()
{
	return _otYAxisSpeed;
}

void StationVM::setOtYAxisSpeed(int value)
{
	if(value != _otYAxisSpeed) {
		_otYAxisSpeed = value;
		Q_EMIT otYAxisSpeedChanged();
	}
}

int StationVM::otZAxisSpeed()
{
	return _otZAxisSpeed;
}

void StationVM::setOtZAxisSpeed(int value)
{
	if(value != _otZAxisSpeed) {
		_otZAxisSpeed = value;
		Q_EMIT otZAxisSpeedChanged();
	}
}
