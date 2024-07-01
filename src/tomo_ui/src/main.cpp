#include <QApplication>
#include <QDebug>
#include <QElapsedTimer>
#include <QIcon>
#include <QPixmap>
#include <QQmlApplicationEngine>

// My include
#include "Dialog/modal_dialogbox.h"
#include "ViewModel/main_vm.h"
#include <ros/ros.h>
#include <signal.h>
// end

void quitRequested(int sig)
{
	ROS_WARN("sigterm #%d\n", sig);
	fflush(stdout);
	QCoreApplication::quit();
}

int main(int argc, char** argv)
{
	qputenv("QT_IM_MODULE", QByteArray("qtvirtualkeyboard"));
#if QT_VERSION < QT_VERSION_CHECK(6, 0, 0)
	QCoreApplication::setAttribute(Qt::AA_EnableHighDpiScaling);
#endif

	QElapsedTimer timer;
	timer.start();
	if(!ros::isInitialized()) {
		ros::init(argc, argv, "tomo_ui", ros::init_options::AnonymousName);
	}
	signal(SIGTERM, quitRequested);
	signal(SIGINT, quitRequested);
	signal(SIGHUP, quitRequested);

	/// Udupa 08Feb'24; Added UI simulation mode
	bool isSim = false;
	if(argc > 1)
		isSim = !strcmp(argv[1], "true");

	QApplication qapp(argc, argv);

	qapp.setOrganizationName("Emage TomO");
	qapp.setOrganizationDomain("@emagegroup.com");
	qapp.setApplicationName("TomO Cartoner");
	qapp.setApplicationDisplayName("TomO Cartoner");
	qapp.setDesktopFileName("Tomo Cartoner");
	qapp.setWindowIcon(QIcon(QPixmap(":/Resources/tomoc.png").scaled(QSize(64, 64))));

	QQmlApplicationEngine engine;
	bool isLoadRviz = true;
	bool isLoadQml	= true;

	try {
		MainVM mainVM(&qapp, argc, argv, isLoadRviz, isSim);

		engine.rootContext()->setContextProperty(QStringLiteral("mainVM"), &mainVM);
		engine.rootContext()->setContextProperty(QStringLiteral("master_app"), mainVM.master_app.data());
		engine.rootContext()->setContextProperty(QStringLiteral("modalDialogBoxVM"), &mainVM.master_app.data()->modalDialog);
		engine.rootContext()->setContextProperty(QStringLiteral("controlBarVM"), mainVM.controlbar_vm.data());
		engine.rootContext()->setContextProperty(QStringLiteral("systemBarVM"), mainVM.systembar_vm.data());
		engine.rootContext()->setContextProperty(QStringLiteral("dashboardVM"), mainVM.master_app.data()->dashboard_vm.data());
		engine.rootContext()->setContextProperty(QStringLiteral("tomoVM"), mainVM.tomo_vm.data());
		engine.rootContext()->setContextProperty(QStringLiteral("headVM"), mainVM.head_vm.data());
		engine.rootContext()->setContextProperty(QStringLiteral("shipperVM"), mainVM.shipper_vm.data());
		engine.rootContext()->setContextProperty(QStringLiteral("pimVM"), mainVM.pim_vm.data());
		engine.rootContext()->setContextProperty(QStringLiteral("settingVM"), mainVM.setting_vm.data());
		engine.rootContext()->setContextProperty(QStringLiteral("notifyVM"), mainVM.notify_vm.data());
		engine.rootContext()->setContextProperty(QStringLiteral("loginVM"), mainVM.login_vm.data());
		engine.rootContext()->setContextProperty(QStringLiteral("systemLogVM"), mainVM.systembar_vm.data()->systemlog_vm.data());
		engine.rootContext()->setContextProperty(QStringLiteral("modeTomoView"), mainVM.tomo_vm.data()->modeTomoView);
		engine.rootContext()->setContextProperty(QStringLiteral("stateTomoView"), mainVM.tomo_vm.data()->stateTomoView);
		engine.rootContext()->setContextProperty(QStringLiteral("stationVM"), mainVM.master_app.data()->station_vm.data());
		engine.rootContext()->setContextProperty(QStringLiteral("productionVM"), mainVM.master_app.data()->production_vm.data());
		engine.rootContext()->setContextProperty(QStringLiteral("ioControlVM"), mainVM.io_control_vm.data());
		engine.rootContext()->setContextProperty(QStringLiteral("imagingClientVM"), mainVM.imaging_client_vm.data());

		if(isLoadQml) {
			const QUrl url(QStringLiteral("qrc:/main.qml"));
			QObject::connect(
				&engine, &QQmlApplicationEngine::objectCreated, &qapp,
				[url](QObject* obj, const QUrl& objUrl) {
					if(!obj && url == objUrl) {
						QCoreApplication::exit(-1);
					}
				},
				Qt::QueuedConnection);
			engine.load(url);
		}

		qDebug() << "------------------ Application initialized in" << timer.elapsed() << "ms -----------------";
		return qapp.exec();
	}
	catch(const std::exception& e) {
		std::cerr << e.what() << '\n';
	}
	return 0;
}
