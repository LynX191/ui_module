#include "main_vm.h"
#include "../Script/Define/struct_def.h"
#include "image_writer.h"

MainVM::MainVM(QApplication* qapp, int argc, char** argv, bool loadRViz, bool isSim, QObject* parent) : QObject(parent)
{

	// Register meta variable custom
	qRegisterMetaType<tVectorU>("tVectorU");
	qRegisterMetaType<tVectorI>("tVectorI");
	qRegisterMetaType<TrackVM*>("TrackVM*");
	qRegisterMetaType<ImageModel*>("ImageModel*");
	qRegisterMetaType<ImageViewVM*>("ImageViewVM*");
	qRegisterMetaType<QList<QStringList>>("QList<QStringList>");
	qRegisterMetaType<std::string>("std::string");

	// Register type custom
	qmlRegisterType<ImageWriter>("ImageWriter", 1, 0, "ImageWriter");
	qmlRegisterType<LoginModel>("DataLogin", 1, 0, "LoginModel");
	qmlRegisterType<NotifyModel>("DataNotify", 1, 0, "NotifyModel");
	qmlRegisterUncreatableType<LoginVM>("DataLogin", 1, 0, "LoginVM", QStringLiteral("Account should not be created in QML"));

	master_app.reset(new MasterApp(isSim, parent));
	controlbar_vm.reset(new ControlBarVM(master_app.data(), parent));
	systembar_vm.reset(new SystemBarVM(master_app.data(), parent));
	tomo_vm.reset(new TomoVM(argc, argv, master_app.data(), qapp, parent));
	head_vm.reset(new HeadVM(master_app.data(), parent));
	shipper_vm.reset(new ShipperVM(master_app.data(), parent));
	pim_vm.reset(new PimVM(master_app.data(), parent));
	setting_vm.reset(new SettingVM(master_app.data(), parent));
	login_vm.reset(new LoginVM(master_app.data(), parent));
	notify_vm.reset(new NotifyVM(master_app.data(), parent));
	io_control_vm.reset(new IoControlVM(master_app.data(), parent));
	imaging_client_vm.reset(new ImagingClientVM(parent));

	_isEnableRviz = loadRViz;

	createConnection();
}

MainVM::~MainVM()
{
	delete controlbar_vm.data();
	delete systembar_vm.data();
	delete login_vm.data();
	delete notify_vm.data();
	delete setting_vm.data();
	delete tomo_vm.data();
	delete head_vm.data();
	delete shipper_vm.data();
	delete pim_vm.data();
	delete io_control_vm.data();
	delete imaging_client_vm.data();
	delete master_app.data();
}

void MainVM::createConnection()
{
	connect(master_app.data(), SIGNAL(powerUpStateChanged(bool)), systembar_vm.data()->systemlog_vm.data(), SLOT(startControl(bool)));
	connect(master_app.data(), SIGNAL(confirmResetExitPass()), controlbar_vm.data(), SLOT(resetExitPass()));
 
	connect(master_app.data(), SIGNAL(killTerminal(int, QString, bool)), systembar_vm.data()->systemlog_vm.data(),
			SIGNAL(sendStateButtonCmd(int, QString, bool)));

	connect(tomo_vm.data(), SIGNAL(setProcessState(QString)), systembar_vm.data(), SLOT(setProcessState(QString)));
	connect(tomo_vm.data(), SIGNAL(setLotState(QString)), systembar_vm.data(), SLOT(setLotState(QString)));

	connect(systembar_vm.data()->systemlog_vm.data(), SIGNAL(setEnableListModesStates(bool)), tomo_vm.data(),
			SLOT(setEnbaleModesStates(bool)));
	connect(&master_app.data()->modalDialog, SIGNAL(setIsCovered(bool)), master_app.data()->dashboard_vm.data(), SLOT(setIsCovered(bool)));

	// connect(login_vm.data(), SIGNAL(superUserActiveChanged(bool)), tomo_vm.data(), SLOT(setSuperUserAct(bool)));
	// connect(login_vm.data(), SIGNAL(superUserActiveChanged(bool)), head_vm.data(), SLOT(setSuperUserAct(bool)));
	// connect(login_vm.data(), SIGNAL(superUserActiveChanged(bool)), pim_vm.data(), SLOT(setSuperUserAct(bool)));
	// connect(login_vm.data(), SIGNAL(superUserActiveChanged(bool)), shipper_vm.data(), SLOT(setSuperUserAct(bool)));

	if(_isEnableRviz) {
		connect(tomo_vm.data(), SIGNAL(requestConnectRviz()), this, SLOT(connectRviz()));
		connect(tomo_vm.data(), SIGNAL(requestDisconnectRviz()), this, SLOT(disconnectRviz()));
		connect(systembar_vm.data()->systemlog_vm.data(), SIGNAL(setRvizReadyToCall(bool)), tomo_vm.data(), SLOT(setRvizReadyToCall(bool)));
	}

	connect(master_app.data()->station_vm.data(), SIGNAL(callIOTab(int)), controlbar_vm.data(), SLOT(setIndexTab(int)));
	connect(master_app.data()->station_vm.data(), SIGNAL(bsSpeedSliderChanged(int, int)), master_app.data(),
			SLOT(updateSpeedSlider(int, int)));
	connect(master_app.data()->station_vm.data(), SIGNAL(tsSpeedSliderChanged(int, int)), master_app.data(),
			SLOT(updateSpeedSlider(int, int)));
	connect(master_app.data()->station_vm.data(), SIGNAL(itSpeedSliderXChanged(int, int)), master_app.data(),
			SLOT(updateSpeedSlider(int, int)));
	connect(master_app.data()->station_vm.data(), SIGNAL(itSpeedSliderYChanged(int, int)), master_app.data(),
			SLOT(updateSpeedSlider(int, int)));
	connect(master_app.data()->station_vm.data(), SIGNAL(afSpeedSliderXChanged(int, int)), master_app.data(),
			SLOT(updateSpeedSlider(int, int)));
	connect(master_app.data()->station_vm.data(), SIGNAL(afSpeedSliderYChanged(int, int)), master_app.data(),
			SLOT(updateSpeedSlider(int, int)));
	connect(master_app.data()->station_vm.data(), SIGNAL(otSpeedSliderXChanged(int, int)), master_app.data(),
			SLOT(updateSpeedSlider(int, int)));
	connect(master_app.data()->station_vm.data(), SIGNAL(otSpeedSliderYChanged(int, int)), master_app.data(),
			SLOT(updateSpeedSlider(int, int)));
	connect(master_app.data()->station_vm.data(), SIGNAL(otSpeedSliderZChanged(int, int)), master_app.data(),
			SLOT(updateSpeedSlider(int, int)));

	connect(master_app.data()->station_vm.data(), SIGNAL(getLimitSlider()), master_app.data(), SLOT(initStationVM()));
}

void MainVM::closeAppFormUI()
{
	master_app.data()->config.writeFileConfig();
	tomo_vm.data()->closeRvizApp();
}

void MainVM::updateRvizWindowState(bool visible)
{
	Q_EMIT sendRVizWindowState(visible);
}

void MainVM::setEnableAllLayouts(bool isEnable)
{
	Q_EMIT sendEnableAllsLayout(isEnable);
}

void MainVM::connectRviz()
{
	if(tomo_vm.data()->rviz_app) {
		connect(this, SIGNAL(sendRVizPanelDisplay(bool)), tomo_vm.data()->rviz_app.get(), SLOT(setRVizPanelDisplay(bool)));
		connect(this, SIGNAL(sendRVizWindowState(bool)), tomo_vm.data()->rviz_app.get(), SLOT(setRVizWindowState(bool)));
	}
}

void MainVM::disconnectRviz()
{
	if(tomo_vm.data()->rviz_app) {
		disconnect(this, SIGNAL(sendRVizPanelDisplay(bool)), tomo_vm.data()->rviz_app.get(), SLOT(setRVizPanelDisplay(bool)));
		disconnect(this, SIGNAL(sendRVizWindowState(bool)), tomo_vm.data()->rviz_app.get(), SLOT(setRVizWindowState(bool)));
	}
}

void MainVM::uiCompleted()
{
	Q_EMIT signalUICompleted();
}

void MainVM::updateRVizPanelDisplay()
{
	Q_EMIT sendRVizPanelDisplay(true);
}
