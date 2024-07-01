#include "recovery.h"
#include <QGuiApplication>
#include <QQmlApplicationEngine>
#include <QQmlContext>

int main(int argc, char* argv[])
{
	QCoreApplication::setAttribute(Qt::AA_EnableHighDpiScaling);

	QGuiApplication app(argc, argv);

	QQmlApplicationEngine engine;
    Recovery recovery;
    engine.rootContext()->setContextProperty(QStringLiteral("recoveryVM"), &recovery);
    recovery.setName(argv);

	engine.load(QUrl(QStringLiteral("qrc:/main.qml")));
	if(engine.rootObjects().isEmpty())
		return -1;

	return app.exec();
}
