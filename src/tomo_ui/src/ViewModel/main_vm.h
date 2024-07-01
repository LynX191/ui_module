#ifndef MAINVM_H
#define MAINVM_H

#pragma once

#include <QApplication>
#include <QObject>
#include <QSharedPointer>

// My include
#include "../Dialog/modal_dialogbox.h"
#include "../Model/login_model.h"
#include "../Model/notify_model.h"
#include "HeadVM/head_vm.h"
#include "ImagingVM/imaging_client_vm.h"
#include "IoVM/io_control_vm.h"
#include "NotifyVM/notify_vm.h"
#include "PimVM/pim_vm.h"
#include "SettingVM/setting_vm.h"
#include "ShipperVM/shipper_vm.h"
#include "TomoVM/tomo_vm.h"
#include "controlbar_vm.h"
#include "master_app.h"
#include "systembar_vm.h"
//  end

class MainVM : public QObject
{
	Q_OBJECT

public:
	MainVM(QApplication* qapp, int argc, char** argv, bool loadRViz, bool isSim, QObject* parent = nullptr);
	~MainVM();

public:
	QScopedPointer<ControlBarVM> controlbar_vm;
	QScopedPointer<SystemBarVM> systembar_vm;
	QSharedPointer<MasterApp> master_app;
	QScopedPointer<HeadVM> head_vm;
	QScopedPointer<ShipperVM> shipper_vm;
	QScopedPointer<PimVM> pim_vm;
	QScopedPointer<ImagingClientVM> imaging_client_vm;
	QScopedPointer<TomoVM> tomo_vm;
	QScopedPointer<IoControlVM> io_control_vm;
	QScopedPointer<SettingVM> setting_vm;
	QScopedPointer<LoginVM> login_vm;
	QScopedPointer<NotifyVM> notify_vm;

private:
	void createConnection();

	bool _isEnableRviz;

Q_SIGNALS:
	void signalUICompleted();
	void sendRVizWindowState(bool);
	void sendRVizPanelDisplay(bool);
	void sendEnableAllsLayout(bool isEnable);

public Q_SLOTS:
	void uiCompleted();
	void connectRviz();
	void disconnectRviz();
	void closeAppFormUI();
	void updateRVizPanelDisplay();
	void setEnableAllLayouts(bool);
	void updateRvizWindowState(bool);
};
#endif	// MAINVM_H
