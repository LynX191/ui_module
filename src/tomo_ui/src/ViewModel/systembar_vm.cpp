#include "systembar_vm.h"

SystemBarVM::SystemBarVM(MasterApp* masterApp, QObject* parent) : master_app{masterApp}, QObject{parent}
{
	systemlog_vm.reset(new SystemLogVM(master_app, parent));
}

SystemBarVM::~SystemBarVM()
{
	delete systemlog_vm.data();
}

QString SystemBarVM::processState()
{
	return _processState;
}

void SystemBarVM::setProcessState(QString processState)
{
	if(processState != _processState) {
		_processState = processState;
		Q_EMIT processStateChanged();
	}
}

QString SystemBarVM::lotState()
{
	return _lotState;
}

void SystemBarVM::setLotState(QString lotState)
{
	if(lotState != _lotState) {
		_lotState = lotState;
		Q_EMIT lotStateChanged();
	}
}
