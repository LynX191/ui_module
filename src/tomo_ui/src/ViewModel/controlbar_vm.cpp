#include "controlbar_vm.h"
#include "../Script/Config/cfg_app.h"
#include "../Script/Utilities/utilities.h"

ControlBarVM::ControlBarVM(MasterApp* masterApp, QObject* parent) : master_app(masterApp), QObject{parent}
{
	_isComponentCompleted = true;
	_playIconStatus		  = false;
}
void ControlBarVM::setAfterUiCreated()
{
	Q_EMIT setConfigTomOView();

	if(master_app->config.tabIndex == DASH_BOARD) {
		master_app->config.tabIndex == PRODUCTION;
		setIndexTab(PRODUCTION);
		master_app->config.writeFileConfig();
	}
	else{
		setIndexTab( master_app->config.tabIndex);
	}
}

ControlBarVM::~ControlBarVM()
{
}

void ControlBarVM::step_btn_click()
{
	if(!master_app)
		return;
	UI_INFO("[ControlBarVM::step_btn_click] master_app->control_comm->sendMessage: r1");
	master_app->control_comm->sendMessage("r1");
	remote_reciever_.publishNext();
}

void ControlBarVM::state_btn_click()
{
	if(!master_app)
		return;

	usleep(500000);
	UI_INFO("[ControlBarVM::state_btn_click] master_app->control_comm->sendMessage: r0");
	master_app->control_comm->sendMessage("r0");
	remote_reciever_.publishContinue();
}

bool ControlBarVM::powerEnabled()
{
	return _powerEnabled;
}
void ControlBarVM::setPowerEnabled(bool value)
{
	if(value != _powerEnabled) {
		_powerEnabled = value;
		Q_EMIT powerEnabledChanged();
	}
}


bool ControlBarVM::stepEnabled()
{
	return _stepEnabled;
}
void ControlBarVM::setStepEnabled(bool value)
{
	if(value != _stepEnabled) {
		_stepEnabled = value;
		Q_EMIT stepEnabledChanged();
	}
}

bool ControlBarVM::homeEnabled()
{
	return _homeEnabled;
}
void ControlBarVM::setHomeEnabled(bool value)
{
	if(value != _homeEnabled) {
		_homeEnabled = value;
		Q_EMIT homeEnabledChanged();
	}
}

bool ControlBarVM::playIconStatus()
{
	return _playIconStatus;
}


bool ControlBarVM::loginIconStatus()
{
	return _loginIconStatus;
}

bool ControlBarVM::settingIconStatus()
{
	return _settingIconStatus;
}

QVector<int> ControlBarVM::inspecVector(){
	return _inspecVector;
}

void ControlBarVM::setInspecVector(QVector<int> value){
	_inspecVector = value;
	inspecVectorChanged();
}

void ControlBarVM::setIndexTab(int index)
{
	Q_EMIT setCurrentIndexTab(index);
}

void ControlBarVM::setPlayIconStatus(bool value)
{
	if(_playIconStatus != value) {
		_playIconStatus = value;
		Q_EMIT playIconStatusChanged();
		setStepEnabled(!value);
	}
}

void ControlBarVM::setLoginIconStatus(bool value)
{
	if(_loginIconStatus != value) {
		_loginIconStatus = value;
		Q_EMIT loginIconStatusChanged();
	}
}

void ControlBarVM::setSettingIconStatus(bool value)
{
	if(_settingIconStatus != value) {
		_settingIconStatus = value;
		Q_EMIT settingIconStatusChanged();
	}
}

void ControlBarVM::playBtnClick(bool value)
{
	if(!master_app)
		return;

	if(!master_app->control_comm) {
		UI_ERROR("ControlBarVM.play: Comm node is invalid");
		return;
	}
	if(value) {
		UI_INFO("[ControlBarVM::playBtnClick] master_app->control_comm->sendMessage: r1");
		master_app->control_comm->sendMessage("r1");
		remote_reciever_.publishContinue();
	}
	else
		remote_reciever_.publishStop();
	setPlayIconStatus(value);
}

void ControlBarVM::powerOnBtnClick(bool powerValue)
{
	if(powerValue) {
		UI_INFO("<<<<<  Bring up system  >>>>>");
		master_app->setPowerUpState(true);
		master_app->station_vm->servo_error = false;
		master_app->setSystemReady(true);
		master_app->setXavierReady(true);
	}
	else
		master_app->modalDialog.getModalDialog(true, "terminal_off_all");
}

void ControlBarVM::tabCurrentChanged(int index)
{
	if(!_isComponentCompleted) {
		return;
	}

	switch(index) {
	case TAB_CONTROL_BAR_INDEX::DASH_BOARD:
		master_app->dashboard_vm->setListModel(true);
		break;
	case TAB_CONTROL_BAR_INDEX::PRODUCTION:
		break;
	case TAB_CONTROL_BAR_INDEX::IO:
		break;
	default:
		break;
	}
	master_app->config.tabIndex = index;
	master_app->config.writeFileConfig();
	setCurrentIndexTab(index);
}

void ControlBarVM::setComponentCompleted()
{
	_isComponentCompleted = true;
}

void ControlBarVM::checkExitPass(QString passInput){
	QString pass;
	QString tempPass	 = QString::fromUtf8(QCryptographicHash::hash(passInput.toUtf8(), QCryptographicHash::Sha3_256));
	QByteArray utf8Bytes = tempPass.toUtf8();
	pass				 = QString::fromLatin1(utf8Bytes.toBase64());
	UI_WARN("[ControlBarVM::checkExitPass] Current pass: %s", convertToStdString(pass).c_str());
	bool checkResult = pass == convertToQString(master_app->config.exitPass);
	UI_WARN("[ControlBarVM::checkExitPass] Compair result: %s", checkResult ? "true" : "false" );
	if(checkResult){
		master_app->exitDoubleClicked();
	}
	else{
		master_app->modalDialog.getModalDialog(false, "wrong_exit");
	}
}

int ControlBarVM::checkChangePass(QString currentPassIn, QString newPassIn, QString confirmNewPassIn){
	QString pass;
	QString tempPass	 = QString::fromUtf8(QCryptographicHash::hash(currentPassIn.toUtf8(), QCryptographicHash::Sha3_256));
	QByteArray utf8Bytes = tempPass.toUtf8();
	pass				 = QString::fromLatin1(utf8Bytes.toBase64());
	bool checkResult = pass == convertToQString(master_app->config.exitPass);
	UI_WARN("[ControlBarVM::checkChangePass] Compair result: %s", checkResult ? "true" : "false" );
	if(!checkResult){
		master_app->modalDialog.getModalDialog(false, "wrong_current");
		return 1;
	}
	if(newPassIn == currentPassIn){
		master_app->modalDialog.getModalDialog(false, "same_current");
		currentPassIn = "";
		return 2;
	}
	if(confirmNewPassIn != newPassIn){
		confirmNewPassIn = "";
		master_app->modalDialog.getModalDialog(false, "wrong_confirm");
		return 3;
	}
	QRegularExpression regex("^(?=.*[a-z])(?=.*[A-Z]).{5,}$");
	QRegularExpressionMatch match = regex.match(newPassIn);
	if(!match.hasMatch()){
		newPassIn = "";
		master_app->modalDialog.getModalDialog(false, "invalid_pass");
		return 4;
	}
	
	QString newPass;
	QString tempNewPass	 = QString::fromUtf8(QCryptographicHash::hash(newPassIn.toUtf8(), QCryptographicHash::Sha3_256));
	QByteArray utf8NewBytes = tempNewPass.toUtf8();
	newPass				 = QString::fromLatin1(utf8NewBytes.toBase64());

	master_app->config.exitPass = convertToStdString(newPass);
	master_app->config.writeFileConfig();
	master_app->modalDialog.getModalDialog(false, "exit_change");
	return 0;
}

void ControlBarVM::resetExitPass(){
	master_app->config.exitPass = "77+977+9P2JR77+9e++/vX5u77+9";
	master_app->config.writeFileConfig();
	master_app->modalDialog.getModalDialog(false, "reset_success");
}