#ifndef STRUCTDEF_H
#define STRUCTDEF_H

#include <QHash>
#include <QImage>
#include <QObject>
#include <QVector>

// My include
// end

// color and position infor of image
struct ImageColor_PositionInfo
{
	int red;
	int green;
	int blue;
	int gray;
	int posX;
	int posY;

	Q_GADGET
	Q_PROPERTY(int ui_red MEMBER red)
	Q_PROPERTY(int ui_green MEMBER green)
	Q_PROPERTY(int ui_blue MEMBER blue)
	Q_PROPERTY(int ui_gray MEMBER gray)
	Q_PROPERTY(int ui_posX MEMBER posX)
	Q_PROPERTY(int ui_posY MEMBER posY)
};

enum DocViewMode
{
	Original = 0,
	Shrink,
	Stretch
};

enum STATIONS
{
	STATION_0,
	STATION_1,
	STATION_2,
	STATION_3,
	STATION_COUNT
};

enum STATION_DOCS
{
	DOC_0,
	DOC_1,
	DOC_2,
	DOC_3,
	DOC_4,
	STATION_DOC_COUNT
};
enum TAB_CONTROL_BAR_INDEX
{
	IO = 0,
	PRODUCTION,
	DASH_BOARD,
	TAB_COUNT
};
enum DASH_BOARD_INDEX
{
	RETURN_NORMAL = 0,
	VISION_CAM,
	HEAD_CAM,
	DASH_BOARD_COUNT
};

enum BUTTON_CONTROL_BAR
{
	PLAY_BTN,
	STEP_BTN,
	STATE_BTN,
	HOME_BTN,
	BTN_COUNT
};

static std::map<BUTTON_CONTROL_BAR, std::string> buttoncontrolbarToString = {
	{PLAY_BTN, "play"}, {STEP_BTN, "step"}, {STATE_BTN, "state"}, {HOME_BTN, "home"}};

enum CAMERAS_FOCUS{
	TOMO_POSE = 0 ,

	TOMO = 10,
	FILLING,
	CLOSING,
	OUTPUT ,


	SHIPPER = 20,
	TSTIM,
	BSTIM,
	FOLDING,

	TRANSFER = 30,
	CARTON,
};

static std::map<CAMERAS_FOCUS, std::string> cameraFocusToString = {
    {TOMO_POSE, "Tomo Pose"},
    {TOMO, "Tomo"},
    {FILLING, "Filling"},
    {CLOSING, "Closing"},
    {OUTPUT, "Output"},
    {SHIPPER, "Shipper"},
    {TSTIM, "TSTIM"},
    {BSTIM, "BSTIM"},
    {FOLDING, "Afolding"},
    {TRANSFER, "Carton Transfer"},
    {CARTON, "Carton"}
};

enum LevelSingal{
	EMAGE = 0,
	ADMINISTRATOR,
	TECHNICIAN,
	OPERATOR,
	LEVELCOUNT
};

static std::map<std::string, int> NameToLevelSignal = {
    {"Emage", EMAGE},
    {"Administrator", ADMINISTRATOR},
    {"Technician", TECHNICIAN},
    {"Operator", OPERATOR}
};
static std::map<int,std::string > LevelSingalToName = {
    {EMAGE ,"Emage" },
    {ADMINISTRATOR ,"Administrator" },
    {TECHNICIAN, "Technician" },
    {OPERATOR ,"Operator" }
};

enum StationOder{
	INVALID_STATION = -1,
	BSTIM_STATION = 1,
	TSTIM_STATION,
	ITPNP_STATION,
	AFOLD_STATION,
	TOMO_STATION,
	CARTON_LOADER_STATION,
	CARTON_TRANSFER_STATION,
	TRAY_TRANSFER_STATION,
	OTPNP_STATION,
	CRC_CONVEYOR,
	ICC_CONVEYOR,
	CSC_CONVEYOR,
	OUTPUT_TRAY_STACK_STATION,
};

static std::map<int,std::string > StationToName = {
    {INVALID_STATION ,"Emage" },
    {BSTIM_STATION, "S01. Bottom Shipper Tray Input Module" },
    {TSTIM_STATION, "S02. Top Shipper Tray Input Module" },
    {ITPNP_STATION, "S03. Input Tray Pick & Place" },
    {AFOLD_STATION, "S04. Automated Folding (Afolding)" },
    {TOMO_STATION, "S05. TomO" },
    {CARTON_LOADER_STATION, "S06. Carton Loader" },
    {CARTON_TRANSFER_STATION,"S07. Carton Transfer" },
    {TRAY_TRANSFER_STATION, "S08. Tray Transfer & Placement" },
    {OTPNP_STATION, "S09. Output Tray Pick & Place" },
    {CRC_CONVEYOR, "S10a. Carton Rotation Conveyor" },
    {ICC_CONVEYOR, "S10b. Infeed Carton Conveyor" },
    {CSC_CONVEYOR, "S10c. Carton Sampling Conveyor" },
    {OUTPUT_TRAY_STACK_STATION,"S11. Output Tray Stack" },
};

enum ProcessState
{
	PROCESS_QUIT = -3,
	PROCESS_PAUSE,
	PROCESS_ERROR,
	PROCESS_INIT,
	PROCESS_READY,
	PROCESS_ONLINE,
	PROCESS_RUNNING,
};
enum LotStates
{
	LOT_IDLE = -1,
	LOT_STARTING,
	LOT_STARTED,
	LOT_ENDING,
	LOT_ENDED
};
//station
enum StationMotor
{
    MOTOR_S1 = 0,
    MOTOR_S2,
    MOTOR_S3X,
    MOTOR_S3Y,
    MOTOR_S4X,
    MOTOR_S4Y,
    MOTOR_S9X,
    MOTOR_S9Y,
    MOTOR_S9Z,
};
enum CurrentAxis
{
    AXIS_S3 = 1,
    AXIS_S4,
    AXIS_S9,
};

enum StateBackupProcess{
	IDLE = 0,
	BUSY,
	ERROR,
};
static std::map<int,std::string > ProcessStateToString = {
{
	{PROCESS_QUIT,"Quit"},
	{PROCESS_PAUSE,"Pause"},
	{PROCESS_ERROR,"Error"},
	{PROCESS_INIT,"Init"},
	{PROCESS_READY,"Ready"},
	{PROCESS_ONLINE,"Online"},
	{PROCESS_RUNNING,"Running"}
}};
static std::map<int,std::string > LotStateToString = {
{
	{LOT_IDLE,"Idle"},
	{LOT_STARTING,"Starting"},
	{LOT_STARTED,"Started"},
	{LOT_ENDING,"Ending"},
	{LOT_ENDED,"Ended"}
}};

enum EndLotTimerAction
{
    TIMER_START = 0,
	TIMER_RESET,
	TIMER_STOP,
	TIMER_RESUME,
};

#endif	// STRUCTDEF_H
