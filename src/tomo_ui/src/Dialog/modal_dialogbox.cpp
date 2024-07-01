#include "modal_dialogbox.h"
#include <ros/ros.h>
#include <yaml-cpp/yaml.h>

ModalDialogBox* p_instance_modal_dialog_box = nullptr;

ModalDialogBox::ModalDialogBox(QObject* parent) : QObject{parent}
{
	ErrorIDToString = ErrorIDToStringDefault;
}

ModalDialogBox::~ModalDialogBox()
{
}

ModalDialogBox* ModalDialogBox::instance()
{
	if(!p_instance_modal_dialog_box)
		p_instance_modal_dialog_box = new ModalDialogBox();
	return p_instance_modal_dialog_box;
}

void ModalDialogBox::getAlarmConfig()
{
	std::string alarmConfig = getenv("HOME") + std::string("/ui_moduleinstall/share/tomo_ui/config/alarm_code.yaml");
	UI_WARN("File alarm config exist: %s",checkPathExists(alarmConfig)? "Yes":"No");
	try {
		YAML::Node alarm_node	= YAML::LoadFile(alarmConfig);
		alarm_data				= alarm_node["ErrorIDToString"].as<std::map<std::string, std::string>>();
		if(ErrorIDToStringDefault.size() != alarm_data.size()){
			UI_ERROR("Alarm map have different, need sync");
		}
		else{
			UI_WARN("Alarm map already synced");
		}
		ErrorIDToString			= alarm_data;
	}
	catch(...){
		ErrorIDToString = ErrorIDToStringDefault;
		UI_ERROR("[ModalDialogBox::getAlarmConfig] Failed to read alarm config file");
	}
}
void ModalDialogBox::getModalDialog(bool isFunctional, std::string warningID)
{
	Q_EMIT setIsCovered(true);
	Q_EMIT popupModalDialog(isFunctional, (int) WARNING_RAISED, convertToQString(warningID), convertToQString(ConditionWarning[warningID]));
}

void ModalDialogBox::getModalDialogQML(bool isFunctional, QString warningID)
{
	Q_EMIT setIsCovered(true);
	Q_EMIT popupModalDialog(isFunctional, (int) WARNING_RAISED, warningID,
							convertToQString(ConditionWarning[convertToStdString(warningID)]));
}

void ModalDialogBox::getLoadingPopup(QString text)
{
	Q_EMIT popupLoading(text);
}

void ModalDialogBox::getClosePopupLoading()
{
	Q_EMIT closePopupLoading();
}

void ModalDialogBox::showErrorMessage(int errorLevel, std::string errorMessage)
{
	if(errorMessage.empty())
		return;

	std::string messageId, message1;
	messageId = errorMessage;
	if(ErrorIDToString.count(messageId)) {
		message1 = ErrorIDToString[messageId];
	}
	else {
		message1  = messageId;
		messageId = "";
	}

	setModalDialogBox((TYPE_MODAL) errorLevel, messageId, message1);

	openModalDialog();
}

void ModalDialogBox::showErrorMessageSlot(int errorLevel, QString errorMessage)
{
	showErrorMessage(errorLevel, convertToStdString(errorMessage));
}

void ModalDialogBox::setModalDialogBox(TYPE_MODAL title, std::string errorID, std::string content)
{
	_title	   = title;
	_messageID = errorID;
	_content   = content;
}

void ModalDialogBox::openModalDialog()
{
	Q_EMIT addToNotifyBar((int) _title, convertToQString(_messageID), convertToQString(_content));
	UI_INFO("[ModalDialogBox] send message ||%s|| to UI",_content.c_str());
}

tVectorI ModalDialogBox::modulesState()
{
	return _modulesState;
}

void ModalDialogBox::setModuleState(QString errorCode, int type)
{
	int module = ErrorIDToModule[convertToStdString(errorCode)];
	if(errorCode == (QString) "im_pause"){
		AlarmToList.push_back({BSTIM_STATION, type});
		AlarmToList.push_back({TSTIM_STATION, type});
	}
	else if(errorCode == (QString) "im_pick_top"){
		AlarmToList.push_back({TSTIM_STATION, type});
	}
	else if(errorCode == (QString) "im_pick_base"){
		AlarmToList.push_back({BSTIM_STATION, type});
	}
	else if(errorCode == (QString) "im_int"){
		AlarmToList.push_back({BSTIM_STATION, type});
		AlarmToList.push_back({TSTIM_STATION, type});
	}
	else if(errorCode == (QString) "om_int_t1"){
		AlarmToList.push_back({OUTPUT_TRAY_STACK_STATION, type});
	}
	else if(errorCode == (QString) "om_int_t2"){
		AlarmToList.push_back({OUTPUT_TRAY_STACK_STATION, type});
	}
	AlarmToList.push_back({module, type});
}
void ModalDialogBox::checkModulesState()
{
	tVectorI temp = {0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0};
	for(int i = 0; i < AlarmToList.size(); i++) {
		int module = AlarmToList[i].first;
		int state  = AlarmToList[i].second;
		if(module <= OUTPUT_TRAY_STACK_STATION && module >= BSTIM_STATION) {
			if(temp[module] > state) {
				temp[module] = state;
			}
		}
	}
	setModulesState(temp);
	AlarmToList.clear();
}
void ModalDialogBox::setModulesState(tVectorI modulesState)
{
	if(modulesState != _modulesState) {
		_modulesState = modulesState;
		Q_EMIT modulesStateChanged();
		Q_EMIT modulesStateConverted(QVector<int>::fromStdVector(_modulesState));
	}
}
