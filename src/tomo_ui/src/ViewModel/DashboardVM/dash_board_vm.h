#ifndef DASHBOARDVM_H
#define DASHBOARDVM_H

#pragma once

// My include
#include "../track_vm.h"
#include <tomo_comm/topic_string.h>
#include <tomo_devices_carton/client_container.h>
#include "../master_app.h"
#include <future>
// end
class MasterApp;

class DashboardVM : public QObject
{
	Q_OBJECT
	Q_PROPERTY(QVector<TrackVM*> listTrackVM READ listTrackVM WRITE setListTrackVM NOTIFY listTrackVMChanged)
	Q_PROPERTY(bool isCovered READ isCovered WRITE setIsCovered NOTIFY isCoveredChanged)
	Q_PROPERTY(QVariantList headModel READ headModel WRITE setHeadModel NOTIFY headModelChanged)
	Q_PROPERTY(QVariantList shipperModel READ shipperModel WRITE setShipperModel NOTIFY shipperModelChanged)
	Q_PROPERTY(QVariantList cartonModel READ cartonModel WRITE setCartonModel NOTIFY cartonModelChanged)
	Q_PROPERTY(QVariantList exceptModel READ exceptModel WRITE setExceptModel NOTIFY exceptModelChanged)

public:
	explicit DashboardVM(QObject* parent = nullptr, MasterApp* master_app = nullptr);
	// explicit DashboardVM(QObject* parent = nullptr);
	~DashboardVM();
	DashboardVM();

public:
	Q_INVOKABLE QObject* trackListAt(int index) const;
	QHash<QString, QSharedPointer<TrackVM>> track_view_models;

	QVector<TrackVM*> listTrackVM();

	void hideTomoPose(bool value);
	void setTrackviewModels(QHash<QString, QSharedPointer<TrackVM>>& tracks);

	bool _isCovered = false;
	void setListModel(bool isTabChanged);
	std::vector<int> findRsyncPID(std::string);
    std::vector<int> rsync_pids;;

private:
	MasterApp* master_app;
	QVector<TrackVM*> _listTrackVM;
	QVariantList _headModel;
	QVariantList _shipperModel;
	QVariantList _cartonModel;
	QVariantList _exceptModel;
	QVariantList extractContent(const std::string& inputText, const std::string& section);
	QStringList _currentSelectList;
	int _currentAction;
	std::string _currentDirectory;
	int _currentCopyOrder = 0;
Q_SIGNALS:
	void isCoveredChanged();
	void setConfigTomOView();
	void listTrackVMChanged();
	void initMaxView(QString tab);
	void hideTomoPoseChanged(bool value);
	void currentIndexOneDocChange(int docIndex);
	void headModelChanged();
	void exceptModelChanged();
	void shipperModelChanged();
	void cartonModelChanged();
	void setExpandDir(int camDir); // 0: head, 1: shipper, 2: pim
	void returnAction(QString action, QString folderName = "" ,int transferred = 1, int total = 1);

public Q_SLOTS:
	bool isCovered();

	void callReturnAction(QString action, QString folderName = "" ,int transferred = 1, int total = 1);
	void setIsCovered(bool);
	void setAfterUiCreated();
	void doubleClickSlots(QString);
	void setListTrackVM(QVector<TrackVM*>&);
	QVariantList headModel();
	QVariantList shipperModel();
	QVariantList cartonModel();
	QVariantList exceptModel();
	void setHeadModel(QVariantList);
	void setShipperModel(QVariantList);
	void setCartonModel(QVariantList);
	void setExceptModel(QVariantList);
	void setSelectList(QString, bool);
	void setActionList(int);
	void setDirectory(QString);
	int checkConnection(int, bool);
};

#endif	// DASHBOARDVM_H
