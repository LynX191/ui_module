#ifndef MODALDIALOGBOX_H
#define MODALDIALOGBOX_H

#include <QObject>
#include <QString>

#include "../Script/Define/struct_def.h"
#include "../Script/Utilities/utilities.h"
#include <tomo_devices_carton/def_server.h>
#include <tomo_utils/tomo_utils.h>

enum TYPE_MODAL
{
	ERROR_RAISED = -3,
	WARNING_RAISED,
	INFO_RAISED,
	ERROR_CLEARED,
	WARNING_CLEARED,
	INFO_CLEARED
};

/// Error message map from sequence, valid as on 05/12; Udupa; Dec'2023
static std::map<std::string, std::string> ErrorIDToStringDefault = {
	{"reset", "Reset Clear all error messages"},

	{"xc1", "Xavier 1 connection error."},
	{"xc2", "Xavier 2 connection error."},
	{"xc3", "Xavier 3 connection error."},
	{"xc4", "Xavier 4 connection error."},

	{"estop1", "Emergency Stop 1 Activated"},
	{"estop2", "Emergency Stop 2 Activated"},
	{"estop3", "Emergency Stop 3 Activated"},
	{"estop4", "Emergency Stop 4 Activated"},
	{"estop5", "Emergency Stop 5 Activated"},
	{"estop6", "Emergency Stop 6 Activated"},
	{"guard1", "Guard Door 1 is open"},
	{"guard1f", "Guard Door 1 - left door is open"},
	{"guard2", "Guard Door 2 is open"},
	{"guard3", "Guard Door 3 is open"},
	{"guard", "Door is unlocked"},
	{"voltage", "Voltage interrupted"},	 // Not used
	{"air", "Air pressure not sufficient"},
	{"airc", "Controlled Air is off"},
	{"init", "Module initialization ERROR"},
	{"vision", "Vision Server is not responding"},
	{"wago", "Wago communication lost"},
	{"delta", "Delta Servo communication lost; Check *** axis"},  // Message text overridden by control
	{"config_1", "XC1 Configuration is invalid. Restore from backup"},
	{"config_2", "XC2 Configuration is invalid. Restore from backup"},
	{"config_3", "XC3 Configuration is invalid. Restore from backup"},
	{"config_4", "XC4 Configuration is invalid. Restore from backup"},
	{"ethercat", "TomO EtherCAT communication failed"},
	{"host", "Host communication failed"},

	{"lot_starting_5", "5 Pack Lot starting"},
	{"lot_starting_30", "30 Pack Lot starting"},
	{"lot_starting_90", "90 Pack Lot starting"},
	{"lot_resuming_5", "5 Pack Lot resuming"},
	{"lot_resuming_30", "30 Pack Lot resuming"},
	{"lot_resuming_90", "90 Pack Lot resuming"},
	{"lot_start", "Lot Started"},
	{"lot_ending", "Lot Ending"},
	{"lot_end", "Lot Ended"},

	{"tomo_top_xfer", "TomO failed to transfer top shipper tray. Please close shipper tray manually"},
	{"tomo_top_close", "TomO failed to close shipper tray. Please close manually"},
	{"tomo_fill", "Carton loading fail"},
	{"tomo_cam", "TomO Head camera connection error"},
	{"tomo_planning_error", "Planning Error"},	// Not used

	{"fm_clear", "Clear Afolding station"},
	{"fm_base", "Bottom Shipper Folding Inspection: Fail."},
	{"fm_top", "Top Shipper Folding Inspection: Fail."},
	{"fm_fail", "Afolding ERROR. Clear Afolding module"},
	{"fm_x", "Afolding X-axis movement error (Servo alarm 'AL###')"},  // Message text overridden by control
	{"fm_y", "Afolding Y-axis movement error (Servo alarm 'AL###')"},  // Message text overridden by control

	{"cm_cam", "Carton Stacker camera error"},
	{"cm_init", "Lot initialization. Clear Carton Transfer station"},
	{"cm_door", "Carton Transfer door open"},
	{"cm_audit_full", "Carton in Carton Sampling Conveyor is Full"},
	{"cm_conveyor", "Stacking Error: Failed to push cartons from conveyor. Remove cartons between conveyor and stack"},
	{"cm_stack", "Stacking Error: Failed to push cartons from stack. Remove cartons between stack and buffer"},
	{"cm_pick1", "Stacking Error: Failed to push cartons to main pick station. Remove cartons from pick station"},
	{"cm_pick2", "Stacking Error: Failed to push cartons to buffer pick station. Remove cartons from buffer pick station"},
	{"cm_jam", "Infeed Carton Conveyor jammed"},
	{"cm_full", "Infeed Carton Conveyor is full"},
	{"cm_line_clear", "Line clearance timed out; Check Conveyor"},

	{"im_cam", "Shipper Tray Vision camera error"},
	{"im_pause", "Input PnP curtain sensor is muted"},
	{"im_pick_top", "Input PnP fail vacuum suction. Failed to pick top shipper tray"},
	{"im_pick_base", "Input PnP fail vacuum suction. Failed to pick bottom shipper tray"},
	{"im_int", "Input PnP curtain sensor interrupted"},
	{"im_x", "Input PnP X-axis movement error (Servo alarm 'AL###')"},	// Message text overridden by control
	{"im_y", "Input PnP Y-axis movement error (Servo alarm 'AL###')"},	// Message text overridden by control
	{"im_z", "Input PnP pneumatic cylinder error. Please check cylinder condition"},
	{"im_fail", "Input PnP fail vacuum suction"},

	{"om_pause_t1", "Curtain Sensor Trolley #1 is muted"},
	{"om_pause_t2", "Curtain Sensor Trolley #2 is muted"},
	{"om_slider", "Output shipper tray stuck on tray transfer roller. Push manually"},
	{"om_push", "Output shipper tray failed to push to pick station. Push manually"},
	{"om_t1_invalid", "Output Trolley #1 stack height check failed. Unload trolley"},
	{"om_t2_invalid", "Output Trolley #2 stack height check failed. Unload trolley"},
	{"om_fail", "Output PnP: Unknown ERROR. Restart UI"},
	{"om_gripper", "Output PnP gripper cylinder error. Check cylinder condition"},
	{"om_load", "Load an empty trolley at Output Tray Stack"},
	{"om_remove_t1", "Lot Initialization: Please clear trolley #1"},
	{"om_remove_t2", "Lot Initialization: Please clear trolley #2"},
	{"om_full", "Trolley is full. Please clear the trolley"},
	{"om_int_t1", "Curtain Sensor Trolley #1 interrupted"},
	{"om_int_t2", "Curtain Sensor Trolley #2 interrupted"},
	{"om_no_trolley", "No empty trolley present at Output Tray Stack"},
	{"om_not_empty_t1", "Newly inserted trolley #1 is not empty."},
	{"om_not_empty_t2", "Newly inserted trolley #2 is not empty."},
	{"om_load_fail_t1", "Trolley 1 lock feedback error"},
	{"om_load_fail_t2", "Trolley 2 lock feedback error"},
	{"om_x", "Output PnP X-axis movement error (Servo alarm 'AL###')"},	 // Message text overridden by control
	{"om_y", "Output PnP Y-axis movement error (Servo alarm 'AL###')"},	 // Message text overridden by control
	{"om_z", "Output PnP Z-axis movement error (Servo alarm 'AL###')"},	 // Message text overridden by control

	{"tm_base", "Bottom Shipper inspection: Fail."},
	{"tm_top", "Top Shipper inspection: Fail."},
	{"tm_door1", "TSTIM Door open"},
	{"tm_door2", "BSTIM Door open"},
	{"tm_fail", "BSTIM/TSTIM motion axis unable to reach expected height"},
	{"tm_base_low", "Low level shipper tray at BSTIM"},
	{"tm_base_empty", "Bottom Shipper Tray Input Module (BSTIM): Empty. Please refill."},
	{"tm_top_low", "Low level shipper tray at TSTIM"},
	{"tm_top_empty", "Top Shipper Tray Input Module (TSTIM): Empty. Please refill."},
	{"tm_base_wrong", "Wrong Bottom Shipper Tray. Please check."},
	{"tm_b", "BSTIM Z-axis movement error (Servo alarm 'AL###')"},	// Message text overridden by control
	{"tm_t", "TSTIM Z-axis movement error (Servo alarm 'AL###')"},	// Message text overridden by control

	{"lm_init", "Shipper tray is present at Carton Loader"},
	{"lm_no_shipper", "Failed to transfer base shipper tray to Carton Loader station; Please place manually"},
	{"lm_push", "Fail to push out shipper tray from Carton Loader station. Please check"},
};

static std::map<std::string, StationOder> ErrorIDToModule = {
	{"tomo_top_xfer", TOMO_STATION},
	{"tomo_top_close", TOMO_STATION},
	{"tomo_fill", TOMO_STATION},
	{"tomo_cam", TOMO_STATION},
	{"tomo_planning_error", TOMO_STATION},

	{"fm_clear", AFOLD_STATION},
	{"fm_base", AFOLD_STATION},
	{"fm_top", AFOLD_STATION},
	{"fm_fail", AFOLD_STATION},
	{"fm_x", AFOLD_STATION},
	{"fm_y", AFOLD_STATION},

	{"cm_cam", CARTON_TRANSFER_STATION},
	{"cm_init", CARTON_TRANSFER_STATION},
	{"cm_door", CARTON_TRANSFER_STATION},
	{"cm_audit_full", CSC_CONVEYOR},
	{"cm_conveyor", CARTON_TRANSFER_STATION},
	{"cm_stack", CARTON_TRANSFER_STATION},
	{"cm_pick1", CARTON_TRANSFER_STATION},
	{"cm_pick2", CARTON_TRANSFER_STATION},
	{"cm_jam", ICC_CONVEYOR},
	{"cm_full", ICC_CONVEYOR},
	{"cm_line_clear", ICC_CONVEYOR},

	{"im_cam", ITPNP_STATION},
	{"im_pause", ITPNP_STATION},
	{"im_pick_top", ITPNP_STATION},
	{"im_pick_base", ITPNP_STATION},
	{"im_int", ITPNP_STATION},
	{"im_x", ITPNP_STATION},
	{"im_y", ITPNP_STATION},
	{"im_z", ITPNP_STATION},
	{"im_fail", ITPNP_STATION},

	{"om_pause_t1", OTPNP_STATION},
	{"om_pause_t2", OTPNP_STATION},
	{"om_slider", TRAY_TRANSFER_STATION},
	{"om_push", TRAY_TRANSFER_STATION},
	{"om_t1_invalid", OUTPUT_TRAY_STACK_STATION},
	{"om_t2_invalid", OUTPUT_TRAY_STACK_STATION},
	{"om_fail", OTPNP_STATION},
	{"om_gripper", OTPNP_STATION},
	{"om_load", OUTPUT_TRAY_STACK_STATION},
	{"om_remove_t1", OUTPUT_TRAY_STACK_STATION},
	{"om_remove_t2", OUTPUT_TRAY_STACK_STATION},
	{"om_full", OUTPUT_TRAY_STACK_STATION},
	{"om_int_t1", OTPNP_STATION},
	{"om_int_t2", OTPNP_STATION},
	{"om_no_trolley", OUTPUT_TRAY_STACK_STATION},
	{"om_not_empty_t1", OUTPUT_TRAY_STACK_STATION},
	{"om_not_empty_t2", OUTPUT_TRAY_STACK_STATION},
	{"om_load_fail_t1", OUTPUT_TRAY_STACK_STATION},
	{"om_load_fail_t2", OUTPUT_TRAY_STACK_STATION},
	{"om_x", OTPNP_STATION},
	{"om_y", OTPNP_STATION},
	{"om_z", OTPNP_STATION},

	{"tm_base", BSTIM_STATION},
	{"tm_top", TSTIM_STATION},
	{"tm_door1", TSTIM_STATION},
	{"tm_door2", BSTIM_STATION},
	{"tm_fail", AFOLD_STATION},
	{"tm_base_low", BSTIM_STATION},
	{"tm_base_empty", BSTIM_STATION},
	{"tm_top_low", TSTIM_STATION},
	{"tm_top_empty", TSTIM_STATION},
	{"tm_base_wrong", BSTIM_STATION},
	{"tm_b", BSTIM_STATION},
	{"tm_t", TSTIM_STATION},

	{"lm_init", CARTON_LOADER_STATION},
	{"lm_no_shipper", CARTON_LOADER_STATION},
	{"lm_push", CARTON_LOADER_STATION},
};
static std::map<std::string, std::string> ConditionWarning = {
	{"exit_click", "Are you sure you want to exit application? - "},
	{"time_fail", "Clock synchronization error. - Restart in Settings tab"},
	{"time_success", "Clocks on all Xavier devices synchronized - Application ready for use."},
	{"wait_server", "Please wait for server ready - Can not use it right now"},
	{"system_lost", "System lost communication - Please check and reset the application"},
	{"bs_home", "Please check BSTIM condition\nAre you sure you want to home? - "},
	{"bs_move", "Please check BSTIM condition\nAre you sure you want to move? - "},
	{"bs_back", "Please check BSTIM condition\nAre you sure you want to move? - "},
	{"bs_forw", "Please check BSTIM condition\nAre you sure you want to move? - "},
	{"ts_home", "Please check TSTIM condition\nAre you sure you want to home? - "},
	{"ts_move", "Please check TSTIM condition\nAre you sure you want to move? - "},
	{"ts_back", "Please check TSTIM condition\nAre you sure you want to move? - "},
	{"ts_forw", "Please check TSTIM condition\nAre you sure you want to move? - "},
	{"it_home", "Please check Input PnP condition\nAre you sure you want to home this axis? - "},
	{"it_home_all", "Please check Input PnP condition\nAre you sure you want to home all axis? - "},
	{"it_move", "Please check Input PnP condition\nAre you sure you want to move? - "},
	{"it_back", "Please check Input PnP condition\nAre you sure you want to move? - "},
	{"it_forw", "Please check Input PnP condition\nAre you sure you want to move? - "},
	{"it_fail", "Please check Input PnP condition - Cylinder is extend. Please retract the cylinder"},
	{"it_pick", "Please check Input PnP condition\nAre you sure you want to pick? - "},
	{"it_place", "Please check Input PnP condition\nAre you sure you want to place? - "},
	{"ot_home", "Please check Output PnP condition\nAre you sure you want to home this axis? - "},
	{"ot_home_all", "Please check Output PnP condition\nAre you sure you want to home all axis? - "},
	{"ot_move", "Please check Output PnP condition\nAre you sure you want to move? - "},
	{"ot_back", "Please check Output PnP condition\nAre you sure you want to move? - "},
	{"ot_forw", "Please check Output PnP condition\nAre you sure you want to move? - "},
	{"terminal_off_all", "Are you sure you want to shutdown all terminal? - "},
	{"terminal_off_1", "Are you sure you want to shutdown this terminal[1]? - "},
	{"terminal_off_2", "Are you sure you want to shutdown this terminal[2]? - "},
	{"terminal_off_3", "Are you sure you want to shutdown this terminal[3]? - "},
	{"terminal_off_4", "Are you sure you want to shutdown this terminal[4]? - "},
	{"terminal_off_5", "Are you sure you want to shutdown this terminal[5]? - "},
	{"id_invalid", "Camera ID wrong format, please check - Camera ID must be 18 characters"},
	{"id_dup", "Camera ID already exist, please check - Camera ID must be unit"},
	{"id_fail", "Change camera ID failed - Can not change to new ID camera, please check connection again before retry"},
	{"id_success", "Camera ID change successful - Camera ID changed, will be apply in next section"},
	{"backup_process_error", "Can not backup folder - Please check connection!"},
	{"restore_process_error", "Can not restore folder - Please check connection!"},
	{"backup_process_busy", "Backup is running - Please wait until the process finishes."},
	{"restore_process_busy", "Restore is running - Please wait until the process finishes."},
	{"process_busy", "Process backup is running - Please wait until the process finishes."},
	{"remove_process_success", "Configuration remove successful - Remove successfully"},
	{"backup_process_success", "Configuration backup successful - Back up successfully"},
	{"list_dir_fail", "Failed to list directory of Xavier 2 - Please check connection before try again"},
	{"not_allow_dir", "Just can save into external media - Please choose another directory"},
	{"sync_time_good", "Successfully sync time for all Xavier - All Xavier's time synced"},
	{"sync_time_bad", "Fail to sync time for all Xavier - All Xavier's time failed to sync"},
	{"init_move", "Ensure initialization prior to using any motion functions for stability - Prioritize initialization for stability "
				  "before employing motion functions"},
	{"parm_reset", "Confirm to reset all parameters for xxxx station to the default values - "},
	{"parm_close", "Confirm to lose changes made to\nthe parameter values - "},
	{"imaging_wrong", "This zip file isn't imaging package - Please choose another one"},
	{"control_failed", "Update control failed - Please try again later"},
	{"ui_wrong",  "This zip file isn't user interface package - Please choose another one"},
	{"control_wrong",  "This zip file isn't control package - Please choose another one"},
	{"wrong_exit", "Wrong exit passwong, can't close application - Please try another one"},
	{"auto_manual", "Are you sure want to change to manual mode? - "},
	{"miss_current", "Current password missing - Current password fields are required"},
	{"miss_new", "New password missing - New password fields are required"},
	{"miss_confirm", "Confirmation password missing - Confirmation password fields are required"},
	{"wrong_current", "Wrong current exit password, can't change password - Please try another one"},
	{"same_current", "New password must have not same with current password, can't change password - Please try another one"},
	{"wrong_confirm", "Confirmation password must have same with current password, can't change password - Please try another one"},
	{"invalid_pass", "Password must have at least 5 characters and contain both uppercase and lowercase letters - Invalid password"},
	{"exit_change", "Exit password change successfully - Will apply immediately"},
	{"exit_reset", "Are you sure want to reset exit password - "},
	{"reset_success", "Exit password reset successfully - Will apply immediately"},
};

class ModalDialogBox : public QObject
{
	Q_OBJECT
public:
	explicit ModalDialogBox(QObject* parent = nullptr);
	~ModalDialogBox();

public:
	static ModalDialogBox* instance();
	void openModalDialog();
	void setModalDialogBox(TYPE_MODAL, std::string, std::string);
	void showErrorMessage(int errorLevel, std::string errorMessage);
	void getAlarmConfig();
	std::map<std::string, std::string> alarm_data;
	std::map<std::string, std::string> ErrorIDToString;

private:
	TYPE_MODAL _title;
	std::string _messageID;
	std::string _content;
	tVectorI _modulesState = {0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0};

	std::vector<std::pair<int, int>> AlarmToList;

Q_SIGNALS:
	void addToNotifyBar(int type, QString messageID, QString content);
	void popupModalDialog(bool isFunctional, int type, QString messageID, QString content);
	// void setModuleState(int typeState, int module);
	void setIsCovered(bool);
	void openEndLotPopup();

	void modulesStateChanged();
	void modulesStateConverted(QVector<int> modulesState);
	void popupLoading(QString text);
	void closePopupLoading();

public Q_SLOTS:
	void showErrorMessageSlot(int errorLevel, QString errorMessage);
	void getModalDialog(bool isFunctional, std::string);
	void getModalDialogQML(bool isFunctional, QString);

	void setModuleState(QString errorCode, int type);
	void checkModulesState();
	tVectorI modulesState();
	void setModulesState(tVectorI modulesState);
	void getLoadingPopup(QString text);
	void getClosePopupLoading();
};

#endif	// MODALDIALOGBOX_H
