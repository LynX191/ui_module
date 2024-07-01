#ifndef PRODUCTION_VM_H
#define PRODUCTION_VM_H
#pragma once

// My include
#include "../../AICore/station_core.h"
#include "../master_app.h"
#include "../track_vm.h"
#include <tomo_comm/topic_string.h>
#include <tomo_devices_carton/client_container.h>

class MasterApp;

class ProductionVM : public QObject
{
	Q_OBJECT
	// Button
	Q_PROPERTY(bool startLotEnabled READ startLotEnabled WRITE setStartLotEnabled NOTIFY startLotEnabledChanged)
	Q_PROPERTY(bool confirmStartLotEnabled READ confirmStartLotEnabled WRITE setConfirmStartLotEnabled NOTIFY confirmStartLotEnabledChanged)
	Q_PROPERTY(bool cancelStartLotEnabled READ cancelStartLotEnabled WRITE setCancelStartLotEnabled NOTIFY cancelStartLotEnabledChanged)
	Q_PROPERTY(bool endLotEnabled READ endLotEnabled WRITE setEndLotEnabled NOTIFY endLotEnabledChanged)
	Q_PROPERTY(bool endLotConfirmEnabled READ endLotConfirmEnabled WRITE setEndLotConfirmEnabled NOTIFY endLotConfirmEnabledChanged)
	Q_PROPERTY(bool auditConveyorEnabled READ auditConveyorEnabled WRITE setAuditConveyorEnabled NOTIFY auditConveyorEnabledChanged)
	Q_PROPERTY(bool resetCountEnabled READ resetCountEnabled WRITE setResetCountEnabled NOTIFY resetCountEnabledChanged)
	Q_PROPERTY(bool startAuditEnabled READ startAuditEnabled WRITE setStartAuditEnabled NOTIFY startAuditEnabledChanged)
	Q_PROPERTY(bool endAuditEnabled READ endAuditEnabled WRITE setEndAuditEnabled NOTIFY endAuditEnabledChanged)
	// End Button

	// Process property
	Q_PROPERTY(bool enabledForRunLot READ enabledForRunLot WRITE setEnabledForRunLot NOTIFY enabledForRunLotChanged)
	Q_PROPERTY(bool enabledForOpenDoor READ enabledForOpenDoor WRITE setEnabledForOpenDoor NOTIFY enabledForOpenDoorChanged)
	Q_PROPERTY(bool firstRunning READ firstRunning WRITE setFirstRunning NOTIFY firstRunningChanged)
	Q_PROPERTY(bool fileValid READ fileValid WRITE setFileValid NOTIFY fileValidChanged)

	// Production Value
	Q_PROPERTY(int recipeValue READ recipeValue WRITE setRecipeValue NOTIFY recipeValueChanged)
	Q_PROPERTY(int productId READ productId WRITE setProductId NOTIFY productIdChanged)
	Q_PROPERTY(QString productName READ productName WRITE setProductName NOTIFY productNameChanged)
	Q_PROPERTY(bool isStartNewLot READ isStartNewLot WRITE setIsStartNewLot NOTIFY isStartNewLotChanged)
	Q_PROPERTY(bool bypassValue READ bypassValue WRITE setBypassValue NOTIFY bypassValueChanged)
	Q_PROPERTY(bool dryRunValue READ dryRunValue WRITE setDryRunValue NOTIFY dryRunValueChanged)
	Q_PROPERTY(bool operationValue READ operationValue WRITE setOperationValue NOTIFY operationValueChanged)
	Q_PROPERTY(int currentModel READ currentModel WRITE setCurrentModel NOTIFY currentModelChanged)
	Q_PROPERTY(int stagesCompleted READ stagesCompleted WRITE setStagesCompleted NOTIFY stagesCompletedChanged)
	Q_PROPERTY(int quantityValue READ quantityValue WRITE setQuantityValue NOTIFY quantityValueChanged)
	Q_PROPERTY(int actualQuantity READ actualQuantity WRITE setActualQuantity NOTIFY actualQuantityChanged)
	Q_PROPERTY(int totalBlowouts READ totalBlowouts WRITE setTotalBlowouts NOTIFY totalBlowoutsChanged)
	Q_PROPERTY(int totalAudit READ totalAudit WRITE setTotalAudit NOTIFY totalAuditChanged)
	Q_PROPERTY(bool soundState READ soundState WRITE setSoundState NOTIFY soundStateChanged)
	// End production value

	// Statistics value
	Q_PROPERTY(QVariantList historyModel READ historyModel NOTIFY historyModelChanged)
	Q_PROPERTY(QVariantList lotDataModel READ lotDataModel NOTIFY lotDataModelChanged)

	Q_PROPERTY(int noBottomShipper READ noBottomShipper WRITE setNoBottomShipper NOTIFY noBottomShipperChanged)
	Q_PROPERTY(int noTopShipper READ noTopShipper WRITE setNoTopShipper NOTIFY noTopShipperChanged)
	Q_PROPERTY(QString lotStartTime READ lotStartTime WRITE setLotStartTime NOTIFY lotStartTimeChanged)
	Q_PROPERTY(int lotStartTimeSec READ lotStartTimeSec WRITE setLotStartTimeSec NOTIFY lotStartTimeSecChanged)
	Q_PROPERTY(QString lotEndTime READ lotEndTime WRITE setLotEndTime NOTIFY lotEndTimeChanged)
	Q_PROPERTY(QString totalDowntime READ totalDowntime WRITE setTotalDowntime NOTIFY totalDowntimeChanged)
	Q_PROPERTY(QString totalRuntime READ totalRuntime WRITE setTotalRuntime NOTIFY totalRuntimeChanged)
	Q_PROPERTY(QString lotDuration READ lotDuration WRITE setLotDuration NOTIFY lotDurationChanged)
	Q_PROPERTY(QString cycleTime READ cycleTime WRITE setCycleTime NOTIFY cycleTimeChanged)
	Q_PROPERTY(int averageCycleTime READ averageCycleTime WRITE setAverageCycleTime NOTIFY averageCycleTimeChanged)
	Q_PROPERTY(int completedTray READ completedTray WRITE setCompletedTray NOTIFY completedTrayChanged)
	Q_PROPERTY(int noLoadedCarton READ noLoadedCarton WRITE setNoLoadedCarton NOTIFY noLoadedCartonChanged)
	Q_PROPERTY(int noIncomingCarton READ noIncomingCarton WRITE setNoIncomingCarton NOTIFY noIncomingCartonChanged)
	Q_PROPERTY(int cartonPerMin READ cartonPerMin WRITE setCartonPerMin NOTIFY cartonPerMinChanged)
	Q_PROPERTY(int avgCartonPerMin READ avgCartonPerMin WRITE setAvgCartonPerMin NOTIFY avgCartonPerMinChanged)

public:
	explicit ProductionVM(QObject* parent = nullptr, MasterApp* master_app = nullptr);
	// explicit ProductionVM(QObject* parent = nullptr);

	~ProductionVM();
	ProductionVM();

	// Property
	// End property

	// Button
	bool _startLotEnabled		 = true;
	bool _confirmStartLotEnabled = true;
	bool _cancelStartLotEnabled	 = true;
	bool _endLotEnabled			 = false;
	bool _endLotConfirmEnabled	 = true;
	bool _auditConveyorEnabled	 = true;
	bool _resetCountEnabled		 = true;
	bool _startAuditEnabled		 = true;
	bool _endAuditEnabled		 = false;
	// End button

	// Process property
	bool _enabledForRunLot	 = true;
	bool _enabledForOpenDoor = true;
	bool _fileValid			 = true;

	// Production Value
	int _recipeValue		= 30;
	int _productId			= -1;
	QString _productName	= "";
	bool _isStartNewLot		= true;
	bool _bypassValue		= false;
	bool _dryRunValue		= false;
	bool _operationValue	= true;
	int _currentModel		= 0;
	tVectorI _endLotProcess = {};
	int _stagesCompleted	= 0;
	int _quantityValue		= 0;
	int _actualQuantity		= 0;
	int _totalBlowouts		= 0;
	int _totalAudit			= 0;
	int _delayTimeValue		= 0;
	bool _soundState		= false;
	bool _statePaused		= false;
	bool _lotStarted		= false;
	bool _firstRunning		= false;

	// Statistics value
	void readFileStatistics();
	void writeFileStatistics(bool);	 // false for only Lot Data, true for both History Cycle Time & Lot Data
	void insertHistoryData();
	void convertToHistoryModel(std::vector<std::string>);
	void convertToLotDataModel(std::vector<std::string>);
	std::vector<std::string> allHistoryCycleTimeData;
	std::vector<std::string> allLotData;
	bool setLotData;  // false: set Lot Data, true: insert Lot Data
	void insertLotData(bool);

	int _noBottomShipper = 0;
	int _noTopShipper	 = 0;
	QString _lotStartTime;
	int _lotStartTimeSec;
	QString _lotEndTime;
	QString _totalDowntime;
	QString _totalRuntime;
	QString _lotDuration;
	QString _cycleTime;
	int _averageCycleTime = 0;
	int _completedTray	  = 0;
	int _noLoadedCarton	  = 0;
	int _noIncomingCarton = 0;
	int _cartonPerMin	  = 0;
	int _avgCartonPerMin  = 0;
	QString _currentDateTime;

private:
	QVariantList _historyModel;
	QVariantList _lotDataModel;

	MasterApp* master_app;

	std::vector<std::string> color_map = {"#4c5052", "#838b8f"};
	std::string last_color;
	std::string last_start_time;

protected:
	string fileStatistics;

Q_SIGNALS:
	void openEndLotPopup();
	void openStartLotConfirm();
	void openEndLotConfirm();
	void openAbortLotConfirm();
	void initializing(bool isRunning);

	void emergencyEvent(bool state, QString nameEvent);
	// Button
	void startLotClickedChanged();
	void startLotEnabledChanged();

	void confirmStartLotClickedChanged();
	void confirmStartLotEnabledChanged();

	void cancelStartLotClickedChanged();
	void cancelStartLotEnabledChanged();

	void endLotClickedChanged();
	void endLotEnabledChanged();

	void endLotConfirmClickedChanged();
	void endLotConfirmEnabledChanged();

	void auditConveyorClickedChanged();
	void auditConveyorEnabledChanged();

	void resetCountClickedChanged();
	void resetCountEnabledChanged();

	void startAuditClickedChanged();
	void startAuditEnabledChanged();

	void endAuditClickedChanged();
	void endAuditEnabledChanged();

	void soundStateChanged();
	// End button

	// Production Value
	void recipeValueChanged();
	void productIdChanged();
	void productNameChanged();
	void isStartNewLotChanged();
	void bypassValueChanged();
	void dryRunValueChanged();
	void operationValueChanged();
	void currentModelChanged();
	void endLotProcessChanged();
	void endLotProcessConverted(QVector<int> endLotVar);
	void stagesCompletedChanged();
	void quantityValueChanged();
	void actualQuantityChanged();
	void totalBlowoutsChanged();
	void totalAuditChanged();
	void delayTimeValueChanged();

	// Process Property
	void enabledForRunLotChanged(bool value);
	void enabledForOpenDoorChanged(bool value);
	void firstRunningChanged();
	void fileValidChanged(bool value);

	// Statistics value
	void historyModelChanged();
	void lotDataModelChanged();

	void noBottomShipperChanged();
	void noTopShipperChanged();
	void lotStartTimeChanged();
	void lotStartTimeSecChanged();
	void lotEndTimeChanged();
	void totalDowntimeChanged();
	void totalRuntimeChanged();
	void lotDurationChanged();
	void cycleTimeChanged();
	void averageCycleTimeChanged();
	void completedTrayChanged();
	void noLoadedCartonChanged();
	void noIncomingCartonChanged();
	void cartonPerMinChanged();
	void avgCartonPerMinChanged();
	void currentDateTimeChanged();

	//
	void switchToProduction();
	void endLotTimerAction(int action); // User enum below to set state of timer
	// TIMER_START = 0,
	// TIMER_RESET,
	// TIMER_STOP,
	// TIMER_RESUME,

public Q_SLOTS:
	// Property
	void showStartLot();
	void showEndLot();
	void showAbortLot();
	void addToHistory();
	int getCurrentTimeProduct();
	void setInitialize(bool);
	// Button
	void startLotClicked();
	void setStartLotEnabled(bool value);
	bool startLotEnabled();

	void confirmStartLotClicked();
	void setConfirmStartLotEnabled(bool value);
	bool confirmStartLotEnabled();

	void cancelStartLotClicked();
	void setCancelStartLotEnabled(bool value);
	bool cancelStartLotEnabled();

	void endLotClicked(bool isConfirm);

	void setEndLotEnabled(bool value);
	bool endLotEnabled();
	void endLotPopup();

	void abortLotConfirmClicked(bool isConfirm);
	void actionBSTIMChoosen(int choosen);  // 0 for purge, 1 for keep and 2 for confirm after choosen action

	void endLotConfirmClicked();
	bool endLotConfirmEnabled();
	void setEndLotConfirmEnabled(bool value);

	void auditConveyorClicked();
	void setAuditConveyorEnabled(bool);
	bool auditConveyorEnabled();

	void resetCountClicked();
	void setResetCountEnabled(bool value);
	bool resetCountEnabled();

	void startAuditClicked();
	void setStartAuditEnabled(bool value);
	bool startAuditEnabled();

	void endAuditClicked();
	void setEndAuditEnabled(bool value);
	bool endAuditEnabled();

	void muteBtnClicked(bool);
	bool soundState();
	void setSoundState(bool value);
	// End button

	// Process Property
	void setEnabledForRunLot(bool value);
	bool enabledForRunLot();

	void setEnabledForOpenDoor(bool value);
	bool enabledForOpenDoor();

	bool firstRunning();
	void setFirstRunning(bool);
	
	void setFileValid(bool value);
	bool fileValid();

	// Production Value
	void setRecipeValue(int value);
	int recipeValue();

	void setProductId(int);
	int productId();

	void setProductName(QString);
	QString productName();

	void setIsStartNewLot(bool isStartNewLot);
	bool isStartNewLot();

	void setBypassValue(bool value);
	bool bypassValue();

	void setDryRunValue(bool value);
	bool dryRunValue();

	void setOperationValue(bool value);
	bool operationValue();
	
	int currentModel();
	void setCurrentModel(int value);

	tVectorI endLotProcess();
	void setEndLotProcess(tVectorI processVector);

	void setStagesCompleted(int value);
	int stagesCompleted();

	void setQuantityValue(int value);
	int quantityValue();

	void setActualQuantity(int value);
	int actualQuantity();

	void setTotalBlowouts(int value);
	int totalBlowouts();

	void setTotalAudit(int value);
	int totalAudit();

	// Statistics Value
	QVariantList historyModel() const;
	QVariantList lotDataModel() const;

	int noBottomShipper();
	int noTopShipper();
	QString lotStartTime();
	int lotStartTimeSec();
	QString lotEndTime();
	QString totalDowntime();
	QString totalRuntime();
	QString lotDuration();
	QString cycleTime();
	int averageCycleTime();
	int completedTray();
	int noLoadedCarton();
	int noIncomingCarton();
	int cartonPerMin();
	int avgCartonPerMin();
	QString currentDateTime();

	void setNoBottomShipper(int);
	void setNoTopShipper(int);
	void setLotStartTime(QString);
	void setLotStartTimeSec(int);
	void setLotEndTime(QString);
	void setTotalDowntime(QString);
	void setTotalRuntime(QString);
	void setLotDuration(QString);
	void setCycleTime(QString);
	void setAverageCycleTime(int);
	void setCompletedTray(int);
	void setNoLoadedCarton(int);
	void setNoIncomingCarton(int);
	void setCartonPerMin(int);
	void setAvgCartonPerMin(int);
	void setCurrentDateTime(QString);
};

#endif	// PRODUCTION_VM_H
