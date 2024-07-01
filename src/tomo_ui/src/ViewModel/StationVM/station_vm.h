#ifndef STATION_VM_H
#define STATION_VM_H
#pragma once

#include "../../Script/Define/struct_def.h"
#include "../master_app.h"
#include <atomic>
#include <condition_variable>
#include <mutex>
#include <thread>
#include <tomo_comm/topic_string.h>
#include <tomo_devices_carton/client_container.h>
#include <tomo_devices_carton/def_delta.h>
#include <tomo_devices_carton/module_base.h>
#include <vector>

#include <tomo_devices_carton/def_server.h>
using namespace tomo_peripherals;

static std::map<int, int> StationToModule = {
	{BSTIM_STATION, TRAY_MODULE},						//
	{TSTIM_STATION, TRAY_MODULE},						//
	{ITPNP_STATION, INPUT_PNP_MODULE},					//
	{AFOLD_STATION, FOLDING_MODULE},					//
	{TOMO_STATION, GRIPPER_MODULE},						//
	{CARTON_LOADER_STATION, LOADER_MODULE},				//
	{CARTON_TRANSFER_STATION, CARTON_TRANSFER_MODULE},	//
	{CRC_CONVEYOR, CARTON_TRANSFER_MODULE},				//
	{ICC_CONVEYOR, CARTON_TRANSFER_MODULE},				//
	{CSC_CONVEYOR, CARTON_TRANSFER_MODULE},				//
	{OTPNP_STATION, OUTPUT_PNP_MODULE},					//
	{TRAY_TRANSFER_STATION, OUTPUT_PNP_MODULE},			//
	{OUTPUT_TRAY_STACK_STATION, OUTPUT_PNP_MODULE},		//
};

class MasterApp;

class StationVM : public QObject
{
	Q_OBJECT
	Q_PROPERTY(int currentAxis READ currentAxis WRITE setCurrentAxis NOTIFY currentAxisChanged)
	Q_PROPERTY(
		int currentMinLimitPosition READ currentMinLimitPosition WRITE setCurrentMinLimitPosition NOTIFY currentMinLimitPositionChanged)
	Q_PROPERTY(
		int currentMaxLimitPosition READ currentMaxLimitPosition WRITE setCurrentMaxLimitPosition NOTIFY currentMaxLimitPositionChanged)
	Q_PROPERTY(int currentMinLimitSpeed READ currentMinLimitSpeed WRITE setCurrentMinLimitSpeed NOTIFY currentMinLimitSpeedChanged)
	Q_PROPERTY(int currentMaxLimitSpeed READ currentMaxLimitSpeed WRITE setCurrentMaxLimitSpeed NOTIFY currentMaxLimitSpeedChanged)
	Q_PROPERTY(QVector<int> moveStateVector READ moveStateVector WRITE setMoveStateVector NOTIFY moveStateVectorChanged)
	Q_PROPERTY(QStringList parametersList READ parametersList WRITE setParametersList NOTIFY parametersListChanged)
	Q_PROPERTY(bool enabledParmButton READ enabledParmButton WRITE setEnabledParmButton NOTIFY enabledParmButtonChanged)

	// Station 1: BSTIM 'bs'
	Q_PROPERTY(int bsSpeedSlider READ bsSpeedSlider WRITE setBsSpeedSlider NOTIFY bsSpeedSliderChanged)
	Q_PROPERTY(int bsPosition READ bsPosition WRITE setBsPosition NOTIFY bsPositionChanged)
	Q_PROPERTY(int bsDistance READ bsDistance WRITE setBsDistance NOTIFY bsDistanceChanged)
	Q_PROPERTY(int bsCurrentPosition READ bsCurrentPosition WRITE setBsCurrentPosition NOTIFY bsCurrentPositionChanged)
	Q_PROPERTY(int bsCurrentSpeed READ bsCurrentSpeed WRITE setBsCurrentSpeed NOTIFY bsCurrentSpeedChanged)

	// Station 2: TSTIM	 'ts'
	Q_PROPERTY(int tsSpeedSlider READ tsSpeedSlider WRITE setTsSpeedSlider NOTIFY tsSpeedSliderChanged)
	Q_PROPERTY(int tsPosition READ tsPosition WRITE setTsPosition NOTIFY tsPositionChanged)
	Q_PROPERTY(int tsDistance READ tsDistance WRITE setTsDistance NOTIFY tsDistanceChanged)
	Q_PROPERTY(int tsCurrentPosition READ tsCurrentPosition WRITE setTsCurrentPosition NOTIFY tsCurrentPositionChanged)
	Q_PROPERTY(int tsCurrentSpeed READ tsCurrentSpeed WRITE setTsCurrentSpeed NOTIFY tsCurrentSpeedChanged)

	// Station 3: Input Tray Pick & Place	'it'
	Q_PROPERTY(int itCurrentAxis READ itCurrentAxis WRITE setItCurrentAxis NOTIFY itCurrentAxisChanged)
	Q_PROPERTY(int itSpeedSliderX READ itSpeedSliderX WRITE setItSpeedSliderX NOTIFY itSpeedSliderXChanged)
	Q_PROPERTY(int itSpeedSliderY READ itSpeedSliderY WRITE setItSpeedSliderY NOTIFY itSpeedSliderYChanged)
	Q_PROPERTY(int itPosition READ itPosition WRITE setItPosition NOTIFY itPositionChanged)
	Q_PROPERTY(int itDistance READ itDistance WRITE setItDistance NOTIFY itDistanceChanged)
	Q_PROPERTY(int itCurrentPosition READ itCurrentPosition WRITE setItCurrentPosition NOTIFY itCurrentPositionChanged)
	Q_PROPERTY(int itCurrentSpeed READ itCurrentSpeed WRITE setItCurrentSpeed NOTIFY itCurrentSpeedChanged)
	Q_PROPERTY(int itXAxisPosition READ itXAxisPosition WRITE setItXAxisPosition NOTIFY itXAxisPositionChanged)
	Q_PROPERTY(int itXAxisSpeed READ itXAxisSpeed WRITE setItXAxisSpeed NOTIFY itXAxisSpeedChanged)
	Q_PROPERTY(int itYAxisPosition READ itYAxisPosition WRITE setItYAxisPosition NOTIFY itYAxisPositionChanged)
	Q_PROPERTY(int itYAxisSpeed READ itYAxisSpeed WRITE setItYAxisSpeed NOTIFY itYAxisSpeedChanged)

	// Station 4: AFolding	'af'
	Q_PROPERTY(int afCurrentAxis READ afCurrentAxis WRITE setAfCurrentAxis NOTIFY afCurrentAxisChanged)
	Q_PROPERTY(int afTrayType READ afTrayType WRITE setAfTrayType NOTIFY afTrayTypeChanged)
	Q_PROPERTY(int afSpeedSliderX READ afSpeedSliderX WRITE setAfSpeedSliderX NOTIFY afSpeedSliderXChanged)
	Q_PROPERTY(int afSpeedSliderY READ afSpeedSliderY WRITE setAfSpeedSliderY NOTIFY afSpeedSliderYChanged)
	Q_PROPERTY(int afPosition READ afPosition WRITE setAfPosition NOTIFY afPositionChanged)
	Q_PROPERTY(int afDistance READ afDistance WRITE setAfDistance NOTIFY afDistanceChanged)
	Q_PROPERTY(int afXAxisPosition READ afXAxisPosition WRITE setAfXAxisPosition NOTIFY afXAxisPositionChanged)
	Q_PROPERTY(int afXAxisSpeed READ afXAxisSpeed WRITE setAfXAxisSpeed NOTIFY afXAxisSpeedChanged)
	Q_PROPERTY(int afYAxisPosition READ afYAxisPosition WRITE setAfYAxisPosition NOTIFY afYAxisPositionChanged)
	Q_PROPERTY(int afYAxisSpeed READ afYAxisSpeed WRITE setAfYAxisSpeed NOTIFY afYAxisSpeedChanged)

	// Station5: TomO Gripper	'tg'
	//  Q_PROPERTY(bool tgGripperAction 	READ tgGripperAction 		WRITE setTgGripperAction 		NOTIFY tgGripperActionChanged)
	Q_PROPERTY(int tgGripperValue READ tgGripperValue WRITE setTgGripperValue NOTIFY tgGripperValueChanged)
	// Q_PROPERTY(bool tgCheckGripper 		READ tgCheckGripper 		WRITE setTgCheckGripper 		NOTIFY tgCheckGripperChanged)
	// Q_PROPERTY(bool tgVacuumOn 			READ tgVacuumOn 			WRITE setTgVacuumOn 			NOTIFY tgVacuumOnChanged)
	// Q_PROPERTY(bool tgCheckVacuum 		READ tgCheckVacuum 			WRITE setTgCheckVacuum 			NOTIFY tgCheckVacuumChanged)
	// Q_PROPERTY(bool tgVacuumBlow 		READ tgVacuumBlow 			WRITE setTgVacuumBlow 			NOTIFY tgVacuumBlowChanged)
	// Q_PROPERTY(bool tgPressureSet 		READ tgPressureSet 			WRITE setTgPressureSet 			NOTIFY tgPressureSetChanged)

	// Station 6: Carton Loader	'cl'
	//  Q_PROPERTY(bool clTrayClamp 		READ clTrayClamp 			WRITE setClTrayClamp 			NOTIFY clTrayClampChanged)
	//  Q_PROPERTY(bool clTrayTilt 			READ clTrayTilt 			WRITE setClTrayTilt 			NOTIFY clTrayTiltChanged)
	//  Q_PROPERTY(bool clTrayTiltAndClamp 	READ clTrayTiltAndClamp 	WRITE setClTrayTiltAndClamp 	NOTIFY clTrayTiltAndClampChanged)
	//  Q_PROPERTY(bool clTrayPushOutBox 	READ clTrayPushOutBox 		WRITE setClTrayPushOutBox 		NOTIFY clTrayPushOutBoxChanged)

	// Station 7: Carton Transfer	'ct'
	//  Q_PROPERTY(bool ctCartonPicked 		READ ctCartonPicked 		WRITE setCtCartonPicked 		NOTIFY ctCartonPickedChanged)
	//  Q_PROPERTY(bool ctWaitStation 		READ ctWaitStation 			WRITE setCtWaitStation 			NOTIFY ctWaitStationChanged)
	//  Q_PROPERTY(bool ctGetStation		READ ctGetStation 			WRITE setCtGetStation 			NOTIFY ctGetStationChanged)
	//  Q_PROPERTY(bool ctWaitFill 			READ ctWaitFill 			WRITE setCtWaitFill 			NOTIFY ctWaitFillChanged)

	// Station 8: Tray Transfer & Placement	'tt'

	// Station 9: Output Tray Pick & Place 'ot'
	Q_PROPERTY(int otCurrentAxis READ otCurrentAxis WRITE setOtCurrentAxis NOTIFY otCurrentAxisChanged)
	Q_PROPERTY(int otSpeedSliderX READ otSpeedSliderX WRITE setOtSpeedSliderX NOTIFY otSpeedSliderXChanged)
	Q_PROPERTY(int otSpeedSliderY READ otSpeedSliderY WRITE setOtSpeedSliderY NOTIFY otSpeedSliderYChanged)
	Q_PROPERTY(int otSpeedSliderZ READ otSpeedSliderZ WRITE setOtSpeedSliderZ NOTIFY otSpeedSliderZChanged)
	Q_PROPERTY(int otPosition READ otPosition WRITE setOtPosition NOTIFY otPositionChanged)
	Q_PROPERTY(int otDistance READ otDistance WRITE setOtDistance NOTIFY otDistanceChanged)
	Q_PROPERTY(int otXAxisPosition READ otXAxisPosition WRITE setOtXAxisPosition NOTIFY otXAxisPositionChanged)
	Q_PROPERTY(int otXAxisSpeed READ otXAxisSpeed WRITE setOtXAxisSpeed NOTIFY otXAxisSpeedChanged)
	Q_PROPERTY(int otYAxisPosition READ otYAxisPosition WRITE setOtYAxisPosition NOTIFY otYAxisPositionChanged)
	Q_PROPERTY(int otYAxisSpeed READ otYAxisSpeed WRITE setOtYAxisSpeed NOTIFY otYAxisSpeedChanged)
	Q_PROPERTY(int otZAxisPosition READ otZAxisPosition WRITE setOtZAxisPosition NOTIFY otZAxisPositionChanged)
	Q_PROPERTY(int otZAxisSpeed READ otZAxisSpeed WRITE setOtZAxisSpeed NOTIFY otZAxisSpeedChanged)

	// Station 10a: Carton Rotation Conveyor 'cr'
	// Station 10b: Infeed Carton Conveyor 'ic'
	// Station 10c: Carton Sampling Conveyor 'cs'
	// Station 11: Output Tray Stack

	// Q_PROPERTY(QString currentString		READ currentString			WRITE setCurrentString			NOTIFY currentStringChanged)

public:
	// explicit StationVM(tClientContainer* client, QObject* parent = nullptr);
	explicit StationVM(QObject* parent = nullptr, MasterApp* master_app = nullptr);
	~StationVM();

	bool servo_error = false;

	std::vector<std::pair<int, int>> motorAxisLimitSpeed = {
		{0, 0},		  // 	0 INVALID_AXIS
		{3000, 50},	  // 	1 AFOLD_X_AXIS S4
		{3000, 50},	  // 	2 AFOLD_Y_AXIS
		{3000, 300},  // 	3 TSTIM_AXIS_Z S2
		{3000, 300},  // 	4 BSTIM_AXIS_Z S1
		{1500, 100},  // 	5 INPUT_PNP_X_AXIS S3
		{1500, 100},  // 	6 INPUT_PNP_Y_AXIS
		{500, 200},	  // 	7 OUTPUT_PNP_X_AXIS S9
		{500, 200},	  // 	8 OUTPUT_PNP_Y_AXIS
		{600, 200},	  // 	9 OUTPUT_PNP_Z_AXIS
	};

	void setCurrentLimit();
	void setSpeedLimit();
	QStringList _parametersList;
	bool _enabledParmButton		 = false;
	int _currentAxis			 = 0;
	int _currentMinLimitPosition = 0;
	int _currentMaxLimitPosition = 0;
	int _currentMinLimitSpeed	 = 0;
	int _currentMaxLimitSpeed	 = 0;
	// Station 1: BSTIM 'bs'
	int _bsSpeedSlider	   = 0;
	int _bsDistance		   = 0;
	int _bsPosition		   = 0;
	int _bsCurrentPosition = 0;
	int _bsCurrentSpeed	   = 0;

	// Station 2: TSTIM 'ts' private members
	int _tsSpeedSlider	   = 0;
	int _tsDistance		   = 0;
	int _tsPosition		   = 0;
	int _tsCurrentPosition = 0;
	int _tsCurrentSpeed	   = 0;

	// Station 3: Input Tray Pick & Place 'it' private members
	int _itCurrentAxis	   = 5;
	int _itSpeedSliderX	   = 0;
	int _itSpeedSliderY	   = 0;
	int _itDistance		   = 0;
	int _itPosition		   = 0;
	int _itCurrentPosition = 0;
	int _itCurrentSpeed	   = 0;
	int _itXAxisPosition   = 0;
	int _itXAxisSpeed	   = 0;
	int _itYAxisPosition   = 0;
	int _itYAxisSpeed	   = 0;

	// Station 4: AFolding 'af' private members
	int _afCurrentAxis	 = 1;
	int _afTrayType		 = 0;
	int _afSpeedSliderX	 = 0;
	int _afSpeedSliderY	 = 0;
	int _afPosition		 = 0;
	int _afDistance		 = 0;
	int _afXAxisPosition = 0;
	int _afXAxisSpeed	 = 0;
	int _afYAxisPosition = 0;
	int _afYAxisSpeed	 = 0;

	// Station 5: TomO Gripper 'tg' private members
	// bool _tgGripperAction;
	int _tgGripperValue;
	// bool _tgGripperConfirm;
	// bool _tgCheckGripper;
	// bool _tgVacuumOn;
	// bool _tgCheckVacuum;
	// bool _tgVacuumBlow;
	// bool _tgPressureSet;

	// Station 6: Carton Loader 'cl' private members
	// bool _clTrayClamp;
	// bool _clTrayTilt;
	// bool _clTrayTiltAndClamp;
	// bool _clTrayPushOutBox;

	// Station 7: Carton Transfer 'ct' private members
	// bool _ctCartonPicked;
	// bool _ctWaitStation;
	// bool _ctGetStation;
	// bool _ctWaitFill;

	// Station 9: Output Tray Pick & Place 'ot' private members
	int _otCurrentAxis	 = 7;
	int _otSpeedSliderX	 = 0;
	int _otSpeedSliderY	 = 0;
	int _otSpeedSliderZ	 = 0;
	int _otPosition		 = 0;
	int _otDistance		 = 0;
	int _otXAxisPosition = 0;
	int _otXAxisSpeed	 = 0;
	int _otYAxisPosition = 0;
	int _otYAxisSpeed	 = 0;
	int _otZAxisPosition = 0;
	int _otZAxisSpeed	 = 0;

	bool waitForState	 = false;
	bool waitHomeAllDone = true;

private:
	MasterApp* master_app;
	int _currentStation	 = 0;
	int _currentPosition = 0;
	std::shared_ptr<std::thread> _stationThreadAxis;
	bool _stateThread		 = false;
	bool _closeStationThread = false;
	std::mutex _stationThreadMutex;
	std::condition_variable _stationThreadcv;
	tClientContainer* _client;
	int stationThreadAxis();
	void close();
	// static std::shared_ptr<tClientContainer> client;
	// tClientContainer& client;
	QVector<int> _tempMoveVector  = {0, 0, 0, 0, 0, 0, 0, 0, 0, 0};
	QVector<int> _moveStateVector = {0, 0, 0, 0, 0, 0, 0, 0, 0, 0};

public Q_SLOTS:
	void goToIoTab(int ioTabIndex);
	void homeAxis(int axis);
	void moveAxis(int axis, int pos, int isAbs, int rpm);
	void stopAxis(int axis);
	void changeReceivePosValMode(bool value);
	void changeThreadState(bool value);
	void startReceivePosValue(int currentStation);
	void getCurPos(int);

	int currentAxis();
	void setCurrentAxis(int);

	int currentMinLimitPosition();
	void setCurrentMinLimitPosition(int);
	int currentMaxLimitPosition();
	void setCurrentMaxLimitPosition(int);

	int currentMinLimitSpeed();
	void setCurrentMinLimitSpeed(int);
	int currentMaxLimitSpeed();
	void setCurrentMaxLimitSpeed(int);
	void setReadyToMove(int, int, bool);

	QVector<int> moveStateVector();
	void setMoveStateVector(QVector<int>);

	void requestParmButtonState(int);
	bool enabledParmButton();
	void setEnabledParmButton(bool);

	void requestParmList(int);
	QStringList parametersList();
	void setParametersList(QStringList);

	int getPosition(QString axis);
	bool getRequireStart(QString nameParameter);

	void setPosition(QString axis, int pos);
	void setParameters(int, QString);
	void setRequireStart(QString nameParameter);

	// Station 1: BSTIM 'bs'
	void bsHomeClicked();
	void bsStopClicked();
	void bsCheckTrayClicked();
	void bsFeedTrayClicked();
	void bsMoveClicked();
	void bsBackwardClicked();
	void bsForwardClicked();

	int bsSpeedSlider();
	int bsPosition();
	int bsDistance();
	int bsCurrentPosition();
	int bsCurrentSpeed();
	void setBsSpeedSlider(int value);
	void setBsPosition(int value);
	void setBsDistance(int value);
	void setBsCurrentPosition(int value);
	void setBsCurrentSpeed(int value);

	// Station 2: TSTIM 'ts'
	void tsHomeClicked();
	void tsStopClicked();
	void tsCheckTrayClicked();
	void tsFeedTrayClicked();
	void tsMoveClicked();
	void tsBackwardClicked();
	void tsForwardClicked();

	int tsSpeedSlider();
	int tsPosition();
	int tsDistance();
	int tsCurrentPosition();
	int tsCurrentSpeed();
	void setTsSpeedSlider(int value);
	void setTsPosition(int value);
	void setTsDistance(int value);
	void setTsCurrentPosition(int value);
	void setTsCurrentSpeed(int value);

	// Station 3: Input Tray Pick & Place 'it' private slots
	void itHomeAllClicked();
	void itStopAllClicked();
	void itPrepareTray();
	void itTransferTray();
	void itHomeClicked();
	void itStopClicked();
	void itMoveClicked();
	void itBackwardClicked();
	void itForwardClicked();

	int itCurrentAxis();
	int itSpeedSliderX();
	int itSpeedSliderY();
	int itPosition();
	int itDistance();
	int itCurrentPosition();
	int itCurrentSpeed();
	int itXAxisPosition();
	int itXAxisSpeed();
	int itYAxisPosition();
	int itYAxisSpeed();
	void setItCurrentAxis(int value);
	void setItSpeedSliderX(int value);
	void setItSpeedSliderY(int value);
	void setItPosition(int value);
	void setItDistance(int value);
	void setItCurrentPosition(int value);
	void setItCurrentSpeed(int value);
	void setItXAxisPosition(int value);
	void setItXAxisSpeed(int value);
	void setItYAxisPosition(int value);
	void setItYAxisSpeed(int value);

	// Station 4: AFolding 'af' private slots
	void afHomeAllClicked();
	void afStopAllClicked();
	void afPrepareClicked();
	void afFoldClicked();
	void afFoldFinishClicked();
	void afHomeClicked();
	void afStopClicked();
	void afMoveClicked();
	void afBackwardClicked();
	void afForwardClicked();

	int afCurrentAxis();
	int afTrayType();
	int afSpeedSliderX();
	int afSpeedSliderY();
	int afPosition();
	int afDistance();
	int afXAxisPosition();
	int afXAxisSpeed();
	int afYAxisPosition();
	int afYAxisSpeed();

	void setAfCurrentAxis(int value);
	void setAfTrayType(int value);
	void setAfSpeedSliderX(int value);
	void setAfSpeedSliderY(int value);
	void setAfPosition(int value);
	void setAfDistance(int value);
	void setAfXAxisPosition(int value);
	void setAfXAxisSpeed(int value);
	void setAfYAxisPosition(int value);
	void setAfYAxisSpeed(int value);

	// Station 5: TomO Gripper 'tg' private slots
	void tgGripperAction();
	void tgCheckGripper();
	void tgVacuumOn();
	void tgCheckVacuum();
	void tgVacuumBlow();
	void tgPressureSet();

	int tgGripperValue();
	void setTgGripperValue(int);

	// void setTgGripperAction(bool value);
	// void setTgGripperValue(int value);
	// void setTgGripperConfirm(bool value);
	// void setTgCheckGripper(bool value);
	// void setTgVacuumOn(bool value);
	// void setTgCheckVacuum(bool value);
	// void setTgVacuumBlow(bool value);
	// void setTgPressureSet(bool value);

	// Station 6: Carton Loader 'cl' private slots
	void clTrayClamp();
	void clTrayTilt();
	void clTrayTiltAndClamp();
	void clTrayPushOutBox();

	// void setClTrayClamp(bool value);
	// void setClTrayTilt(bool value);
	// void setClTrayTiltAndClamp(bool value);
	// void setClTrayPushOutBox(bool value);

	// Station 7: Carton Transfer 'ct' private slots
	void ctCartonPicked();
	void ctWaitStation();
	void ctGetStation();
	void ctWaitFill();

	// void setCtCartonPicked(bool value);
	// void setCtWaitStation(bool value);
	// void setCtGetStation(bool value);
	// void setCtWaitFill(bool value);

	// Station 8: Tray Transfer & Placement	'tt'

	// Station 9: Output Tray Pick & Place 'ot' private slots
	void otHomeAllClicked();
	void otStopAllClicked();
	void otTransferClicked();
	void otHomeClicked();
	void otStopClicked();
	void otMoveClicked();
	void otBackwardClicked();
	void otForwardClicked();

	int otCurrentAxis();
	int otSpeedSliderX();
	int otSpeedSliderY();
	int otSpeedSliderZ();
	int otPosition();
	int otDistance();
	int otXAxisPosition();
	int otXAxisSpeed();
	int otYAxisPosition();
	int otYAxisSpeed();
	int otZAxisPosition();
	int otZAxisSpeed();
	void setOtCurrentAxis(int value);
	void setOtSpeedSliderX(int value);
	void setOtSpeedSliderY(int value);
	void setOtSpeedSliderZ(int value);
	void setOtPosition(int value);
	void setOtDistance(int value);
	void setOtXAxisPosition(int value);
	void setOtXAxisSpeed(int value);
	void setOtYAxisPosition(int value);
	void setOtYAxisSpeed(int value);
	void setOtZAxisPosition(int value);
	void setOtZAxisSpeed(int value);

Q_SIGNALS:

	void callIOTab(int);
	void setTabIndexIO(int ioTabIndex);
	void getLimitSlider();

	void currentAxisChanged();
	void currentMinLimitPositionChanged();
	void currentMaxLimitPositionChanged();
	void currentMinLimitSpeedChanged();
	void currentMaxLimitSpeedChanged();
	void moveStateVectorChanged(QVector<int> stateVector);
	void parametersListChanged();
	void enabledParmButtonChanged();

	// Station 1: BSTIM 'bs'
	void bsSpeedSliderChanged(int, int);
	void bsPositionChanged();
	void bsDistanceChanged();
	void bsCurrentPositionChanged();
	void bsCurrentSpeedChanged();

	// Station 2: TSTIM 'ts' signals
	void tsSpeedSliderChanged(int, int);
	void tsPositionChanged();
	void tsDistanceChanged();
	void tsCurrentPositionChanged();
	void tsCurrentSpeedChanged();

	// Station 3: Input Tray Pick & Place 'it' signals
	void itCurrentAxisChanged(int, int);
	void itSpeedSliderXChanged(int, int);
	void itSpeedSliderYChanged(int, int);
	void itPositionChanged();
	void itDistanceChanged();
	void itCurrentPositionChanged();
	void itCurrentSpeedChanged();
	void itXAxisPositionChanged();
	void itXAxisSpeedChanged();
	void itYAxisPositionChanged();
	void itYAxisSpeedChanged();

	// Station 4: AFolding 'af' signals
	void afCurrentAxisChanged(int, int);
	void afTrayTypeChanged();
	void afSpeedSliderXChanged(int, int);
	void afSpeedSliderYChanged(int, int);
	void afPositionChanged();
	void afDistanceChanged();
	void afXAxisPositionChanged();
	void afXAxisSpeedChanged();
	void afYAxisPositionChanged();
	void afYAxisSpeedChanged();

	// Station 5: TomO Gripper 'tg' signals
	void tgGripperActionChanged();
	void tgGripperValueChanged();
	void tgCheckGripperChanged();
	void tgVacuumOnChanged();
	void tgCheckVacuumChanged();
	void tgVacuumBlowChanged();
	void tgPressureSetChanged();

	// Station 6: Carton Loader 'cl' signals
	// void clTrayClampChanged();
	// void clTrayTiltChanged();
	// void clTrayTiltAndClampChanged();
	// void clTrayPushOutBoxChanged();

	// Station 7: Carton Transfer 'ct' signals
	// void ctCartonPickedChanged();
	// void ctWaitStationChanged();
	// void ctGetStationChanged();
	// void ctWaitFillChanged();

	// Station 9: Output Tray Pick & Place 'ot' signals
	void otCurrentAxisChanged(int, int);
	void otSpeedSliderXChanged(int, int);
	void otSpeedSliderYChanged(int, int);
	void otSpeedSliderZChanged(int, int);
	void otPositionChanged();
	void otDistanceChanged();
	void otXAxisPositionChanged();
	void otXAxisSpeedChanged();
	void otYAxisPositionChanged();
	void otYAxisSpeedChanged();
	void otZAxisPositionChanged();
	void otZAxisSpeedChanged();
};
#endif	// STATION_VM_H
