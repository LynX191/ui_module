#include "recovery.h"
#include <QCoreApplication>
#include <QTimer>
#include <thread>

Recovery::Recovery(QObject* parent) : QObject(parent)
{
}

Recovery::~Recovery() {
    //
}

void Recovery::setName(char** argv)
{
	if(sizeof(argv) / sizeof(*argv) == 0)
		return;
	_name = argv[1];
	QTimer::singleShot(500, [=]() { process(); });
}

void Recovery::process()
{
	std::thread([this]() {
        std::string cmd = std::string("sudo bash ") + getenv("HOME") + std::string("/ui_moduleinstall/share/tomo_ui/config/command/backup.sh -s ");
		cmd.append(_name);
		if(system(cmd.c_str()) == 0) {
            Q_EMIT popupModalDialog(false, -1, "Recovery successful", "Recovery successful - Please restart UI");
		}
		else {
            Q_EMIT popupModalDialog(false, -3, "Recovery fail", "Recovery fail - Contact admin");
		}
	}).detach();
}

void Recovery::close()
{
	// Run Ui
	std::thread([this]() {
		std::string cmd = getenv("HOME") + std::string("/tomo_config/openUI.exp");
        cmd.insert(0, "expect ");
		system(cmd.c_str());
	}).detach();
	QCoreApplication::exit(-1);
}
