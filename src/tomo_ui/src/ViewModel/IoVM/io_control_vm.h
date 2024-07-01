#pragma once

#include <QStringList>
#include <QVariantList>
#include <QVariantMap>

#include "../master_app.h"

#include <algorithm>
#include <atomic>
#include <condition_variable>
#include <mutex>
#include <thread>
#include <tomo_devices_carton/def_wago.h>
#include <vector>
#include <yaml-cpp/yaml.h>

using namespace tomo_peripherals;

struct ModulePin
{
	int module;
	int pin;

	ModulePin(int mod, int p) : module(mod), pin(p)
	{
	}
};

static std::vector<std::pair<std::vector<int>, std::vector<int>>> IOCondition = {
	{{CARTON_TRANSFER_EXT}, {CARTON_STACK_PUSHER_EXT, CARTON_STACK_BUFFER_PUSHER_EXT}},  // 0
	{{CARTON_TRANSFER_EXT_STATE}, {CARTON_STACK_PUSHER_EXT, CARTON_STACK_BUFFER_PUSHER_EXT}}, // 1

	{{CARTON_STACK_PUSHER_EXT}, {CARTON_TRANSFER_EXT, CARTON_PICK_STATION_PUSHER_EXT}}, // 2
	{{CARTON_STACK_PUSHER_EXT_STATE}, {CARTON_TRANSFER_EXT, CARTON_PICK_STATION_PUSHER_EXT}}, // 3
	{{CARTON_STACK_BUFFER_PUSHER_EXT}, {CARTON_TRANSFER_EXT, CARTON_PICK_STATION_PUSHER_EXT}}, // 4
	{{CARTON_STACK_BUFFER_PUSHER_EXT_STATE}, {CARTON_TRANSFER_EXT, CARTON_PICK_STATION_PUSHER_EXT}}, // 5

	{{CARTON_STACK_PUSHER_EXT, CARTON_STACK_BUFFER_PUSHER_EXT}, {CARTON_CONV_STOPPER_EXT, CARTON_PICK_STATION_PUSHER_EXT, CARTON_BUFFER_PICK_STATION_PUSHER_RET}}, // 6
	{{TUCKER_FINGER_EXT_STATE, CARTON_STACK_BUFFER_PUSHER_EXT_STATE}, {CARTON_CONV_STOPPER_EXT, CARTON_PICK_STATION_PUSHER_EXT, CARTON_BUFFER_PICK_STATION_PUSHER_RET}}, // 7
	{{CARTON_STACK_PUSHER_EXT, CARTON_STACK_BUFFER_PUSHER_EXT_STATE}, {CARTON_CONV_STOPPER_EXT, CARTON_PICK_STATION_PUSHER_EXT, CARTON_BUFFER_PICK_STATION_PUSHER_RET}}, // 8
	{{CARTON_STACK_BUFFER_PUSHER_EXT, TUCKER_FINGER_EXT_STATE}, {CARTON_CONV_STOPPER_EXT, CARTON_PICK_STATION_PUSHER_EXT, CARTON_BUFFER_PICK_STATION_PUSHER_RET}}, // 9

	{{CARTON_CONV_STOPPER_EXT, CARTON_STACK_PUSHER_EXT}, {CARTON_STACK_BUFFER_PUSHER_EXT}}, // 10
	{{CARTON_CONV_STOPPER_EXT_STATE, TUCKER_FINGER_EXT_STATE}, {CARTON_STACK_BUFFER_PUSHER_EXT}}, // 11
	{{CARTON_CONV_STOPPER_EXT, TUCKER_FINGER_EXT_STATE}, {CARTON_STACK_BUFFER_PUSHER_EXT}}, // 12
	{{CARTON_STACK_PUSHER_EXT, CARTON_CONV_STOPPER_EXT_STATE}, {CARTON_STACK_BUFFER_PUSHER_EXT}}, // 13
	{{CARTON_BUFFER_PICK_STATION_PUSHER_RET, CARTON_STACK_PUSHER_EXT},{CARTON_STACK_BUFFER_PUSHER_EXT}}, // 14
	{{CARTON_BUFFER_PICK_STATION_PUSHER_RET_STATE, TUCKER_FINGER_EXT_STATE}, {CARTON_STACK_BUFFER_PUSHER_EXT}}, // 15
	{{CARTON_BUFFER_PICK_STATION_PUSHER_RET, TUCKER_FINGER_EXT_STATE}, {CARTON_STACK_BUFFER_PUSHER_EXT}}, // 16
	{{CARTON_STACK_PUSHER_EXT, CARTON_BUFFER_PICK_STATION_PUSHER_RET_STATE}, {CARTON_STACK_BUFFER_PUSHER_EXT}}, // 17

	{{CARTON_BUFFER_PICK_STATION_PUSHER_RET, CARTON_STACK_BUFFER_PUSHER_EXT}, {CARTON_STACK_PUSHER_EXT}}, // 18
	{{CARTON_BUFFER_PICK_STATION_PUSHER_RET_STATE, CARTON_STACK_BUFFER_PUSHER_EXT_STATE}, {CARTON_STACK_PUSHER_EXT}}, // 19
	{{CARTON_BUFFER_PICK_STATION_PUSHER_RET, CARTON_STACK_BUFFER_PUSHER_EXT_STATE}, {CARTON_STACK_PUSHER_EXT}}, // 20
	{{CARTON_STACK_BUFFER_PUSHER_EXT, CARTON_BUFFER_PICK_STATION_PUSHER_RET_STATE}, {CARTON_STACK_PUSHER_EXT}}, // 21
	{{CARTON_CONV_STOPPER_EXT, CARTON_STACK_BUFFER_PUSHER_EXT}, {CARTON_STACK_PUSHER_EXT}}, // 22
	{{CARTON_CONV_STOPPER_EXT_STATE, CARTON_STACK_BUFFER_PUSHER_EXT_STATE}, {CARTON_STACK_PUSHER_EXT}}, // 23
	{{CARTON_CONV_STOPPER_EXT, CARTON_STACK_BUFFER_PUSHER_EXT_STATE}, {CARTON_STACK_PUSHER_EXT}}, // 24
	{{CARTON_STACK_BUFFER_PUSHER_EXT, CARTON_CONV_STOPPER_EXT_STATE}, {CARTON_STACK_PUSHER_EXT}}, // 25

	{{CARTON_PICK_STATION_PUSHER_EXT},{CARTON_STACK_PUSHER_EXT,CARTON_STACK_BUFFER_PUSHER_EXT}}, // 26

	{{TUCKER_RET},{TUCKER_FINGER_EXT}}, // 27
	{{TUCKER_RET_STATE},{TUCKER_FINGER_EXT}}, // 28
	{{TRAY_FOLD_CLAMP_1_EXT},{TUCKER_FINGER_EXT}}, // 29
	{{TRAY_FOLD_CLAMP_1_EXT_STATE},{TUCKER_FINGER_EXT}}, // 30

	{{TUCKER_FINGER_EXT},{TUCKER_RET,TRAY_FOLD_CLAMP_1_EXT}},  // 31
	{{TUCKER_FINGER_EXT_STATE},{TUCKER_RET,TRAY_FOLD_CLAMP_1_EXT}}, // 32

	{{TRAY_TILT_UP},{SHIPPER_PUSHER_EXT}}, // 33
	{{TRAY_TILT_UP_STATE},{SHIPPER_PUSHER_EXT}}, // 34

	{{SHIPPER_PUSHER_EXT},{TRAY_TILT_UP}}, // 35
	{{SHIPPER_PUSHER_EXT_STATE},{TRAY_TILT_UP}}, // 36

	{{SHIPPER_PUSHER_EXT},{TRAY_TILT_UP}}, // 35
	{{SHIPPER_PUSHER_EXT_STATE},{TRAY_TILT_UP}}, // 36

};

static std::vector<std::pair<int, std::vector<int>>> ConditionFromInput = {
	{TRAY_TILT_UP_STATE, {34}},
	{CARTON_BUFFER_PICK_STATION_PUSHER_RET_STATE, {15, 17, 19, 21}},
	{CARTON_TRANSFER_EXT_STATE, {1}},
	{TUCKER_FINGER_EXT_STATE, {7, 9, 11, 12, 15, 16, 32}},
	{CARTON_STACK_BUFFER_PUSHER_EXT_STATE, {5, 7, 8, 19, 20, 23, 24}},
	{CARTON_STACK_PUSHER_EXT_STATE, {3}},
	{SHIPPER_PUSHER_EXT_STATE, {36}},
	{CARTON_CONV_STOPPER_EXT_STATE, {11, 13, 23, 25}},
	{TRAY_FOLD_CLAMP_1_EXT_STATE, {30}}
};

static std::vector<std::pair<int, std::vector<int>>> ConditionFromOutput = {
	{TRAY_FOLD_CLAMP_1_EXT, {29}},
	{CARTON_TRANSFER_EXT, {0}},
	{TRAY_TILT_UP, {33}},
	{SHIPPER_PUSHER_EXT, {35}},
	{CARTON_PICK_STATION_PUSHER_EXT, {26}},
	{CARTON_BUFFER_PICK_STATION_PUSHER_RET, {14,16,18,20}},
	{CARTON_STACK_BUFFER_PUSHER_EXT, {4, 6, 9, 18, 21, 22, 25}},
	{CARTON_STACK_PUSHER_EXT, {2,6,8,10,13,14,17}},
	{TUCKER_RET, {27}},
	{TUCKER_FINGER_EXT, {31}},
	{CARTON_CONV_STOPPER_EXT, {10,12,22,24}},
	{CARTON_CONV_STOPPER_RET,{}}
};

class IoControlVM : public QObject
{
	Q_OBJECT
	Q_PROPERTY(QVariantMap outputModelMap READ getOutputModelMap NOTIFY outputModelMapChanged)
	Q_PROPERTY(QVariantMap inputModelMap READ getInputModelMap NOTIFY inputModelMapChanged)
	Q_PROPERTY(QVariantMap outputItemModelMap READ getOutputItemModelMap NOTIFY outputItemModelMapChanged)
	Q_PROPERTY(QVariantMap inputItemModelMap READ getInputItemModelMap NOTIFY inputItemModelMapChanged)
	Q_PROPERTY(QVariantMap ioModelMap READ getIOModelMap NOTIFY ioModelMapChanged)
	Q_PROPERTY(QStringList inputAnalogMap READ getInputAnalogMap NOTIFY inputAnalogMapChanged)
	Q_PROPERTY(QStringList titleMap READ getTitleMap NOTIFY titleMapChanged)
	// Q_PROPERTY(bool ioConnected READ getIOConnected NOTIFY ioConnectedChanged)

	Q_PROPERTY(QVector<int> outputData READ outputData NOTIFY outputDataChanged)
	Q_PROPERTY(QVector<int> inputData READ inputData NOTIFY inputDataChanged)
	Q_PROPERTY(QVector<int> outputEnable	READ outputEnable	NOTIFY outputEnableChanged)

	Q_PROPERTY(bool inputPnpState READ inputPnpState WRITE setInputPnpState NOTIFY inputPnpStateChanged)

public:
	explicit IoControlVM(MasterApp* masterApp, QObject* parent = nullptr);
	~IoControlVM();

	bool _guardDoorState;
	bool _inputPnpState = false;

	bool _gd1;
	bool _gd2;
	bool _gd3;
	bool _gd4;
	bool _es1;
	bool _es2;
	bool _es3;
	bool _es4;
	bool _es5;
	bool _es6;

	QVector<int> inputCondition = {0, 0, 0, 0};
	bool isTimeOutInput			= false;
public Q_SLOTS:
	void changeReceiveSignalMode(bool value);
	void changeThreadState(bool value);
	void getYamlIOput();
	QVariantMap getOutputModelMap() const;
	QVariantMap getInputModelMap() const;
	QVariantMap getIOModelMap() const;
	QVariantMap getOutputItemModelMap() const;
	QVariantMap getInputItemModelMap() const;
	QStringList getInputAnalogMap() const;
	QStringList getTitleMap() const;
	QVector<int> outputData();
	QVector<int> inputData();
	QVector<int> outputEnable();
	void sendOutput(int module, int pin, int state);
	// bool getIOConnected();
	void setAfterUiCreated();
	void setFilterData(QString);
	bool inputPnpState();
	void setInputPnpState(bool);
	void timeoutInput(bool);
	void inputAccepted(QString);
	void setCompletedIO(bool);

Q_SIGNALS:
	void guardDoorStateChanged(bool state);
	void inputPnpStateChanged(bool state);

	void titleMapChanged();
	void outputModelMapChanged();
	void inputModelMapChanged();
	void ioModelMapChanged();
	void outputItemModelMapChanged();
	void inputItemModelMapChanged();
	void inputAnalogMapChanged();
	// void ioConnectedChanged();
	void outputDataChanged();
	void inputDataChanged();
	void outputEnableChanged();

protected:
private Q_SLOTS:
	void updateOutputData(tVectorU&);
	void updateInputData(tVectorU&);
	void updateInputAnalog(tVectorI);
	void updateOutputEnable(QVector<int>);

private:
	MasterApp* master_app;
	tClientContainer& client;
	QStringList m_titleList;
	QVariantMap m_outputModelMap;
	QVariantMap m_inputModelMap;
	QVariantMap m_ioModelMap;
	QVariantMap m_outputItemModelMap;
	QVariantMap m_inputItemModelMap;
	QStringList m_inputAnalogMap;

	QVector<int> _outputData;
	QVector<int> _inputData;
	QVector<int> _outputEnable;
	std::string _filterData;
	std::vector<int> dataInputCondition;
	std::vector<int> dataOutputCondition;
	bool _inputAccepted = false;
	bool _isCompletedO = false;
	bool _isCompletedI = false;
	// bool m_ioConnected;
	YAML::Node yamlData;
	tVectorU charVector;

	std::shared_ptr<std::thread> _IOThread;
	bool _stateThread	= true;
	bool _closeIOThread = false;
	bool _ioWriting;
	std::mutex _ioThreadMutex;
	std::condition_variable _ioThreadcv;

	int ioThread();
	void close();
	void safetyLock();
};
