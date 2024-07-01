#ifndef NOTIFY_VM_H
#define NOTIFY_VM_H

#pragma once

#include <QDir>
#include <QHash>
#include <QIODevice>
#include <QString>
#include <QVector>
#include <iostream>

#include "../master_app.h"
#include <fstream>
#include <yaml-cpp/yaml.h>
struct NotifyFeature
{
	QString type;
	QString messID;
	QString content;
	QString time;
};

class NotifyVM : public QObject
{
	Q_OBJECT
public:
	bool setItemAt(int, const NotifyFeature&);
	explicit NotifyVM(MasterApp* masterApp, QObject* parent = nullptr);
	QVector<NotifyFeature> items() const;

private:
	MasterApp* master_app;
	void readFileNotify();
	QString fileNotiBarLog;
	QString logDir = QDir::homePath() + "/tomo_stats/notify_log/";

	QVector<QHash<QString, QString>> allNotify;
	QVector<NotifyFeature> mNotify;
	int numOfNoti;

Q_SIGNALS:
	void preRemoveItem(int);
	void postRemoveItem();

	void prePrependItem();
	void postPrependItem();

public Q_SLOTS:
	void writeFileNotify();
	void addNotifyToDatabase(QString, QString, QString, QString);
	void checkDeleteNotifyInDatabase(QString);
};

#endif	// NOTIFY_VM_H
