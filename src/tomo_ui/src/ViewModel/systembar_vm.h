#ifndef SYSTEMBARVM_H
#define SYSTEMBARVM_H

#pragma once
#include "../Script/Utilities/utilities.h"
#include "SystemLogVM/system_log_vm.h"
#include "master_app.h"

class SystemBarVM : public QObject
{
	Q_OBJECT
	Q_PROPERTY(QString processState READ processState WRITE setProcessState NOTIFY processStateChanged)
	Q_PROPERTY(QString lotState READ lotState WRITE setLotState NOTIFY lotStateChanged)

public:
	explicit SystemBarVM(MasterApp* masterApp, QObject* parent = nullptr);
	~SystemBarVM();
	SystemBarVM();

	QScopedPointer<SystemLogVM> systemlog_vm;

public:
	QString _processState = "Init";
	QString _lotState	  = "Idle";

private:
	MasterApp* master_app;

Q_SIGNALS:
	void lotStateChanged();
	void processStateChanged();
	void sendStateRvizWindow(bool);

public Q_SLOTS:
	QString processState();
	void setProcessState(QString);
	QString lotState();
	void setLotState(QString);
};

#endif	// SYSTEMBARVM_H
