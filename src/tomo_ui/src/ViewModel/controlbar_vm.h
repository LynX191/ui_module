#ifndef CONTROLBARVM_H
#define CONTROLBARVM_H

#pragma once

#include <QByteArray>
#include <QCryptographicHash>
#include <QHash>
#include <QRegularExpression>
#include "master_app.h"
#include <rviz_visual_tools/remote_reciever.h>

class ControlBarVM : public QObject
{
	Q_OBJECT
	Q_PROPERTY(bool playIconStatus READ playIconStatus WRITE setPlayIconStatus NOTIFY playIconStatusChanged)
	Q_PROPERTY(bool loginIconStatus READ loginIconStatus WRITE setLoginIconStatus NOTIFY loginIconStatusChanged)
	Q_PROPERTY(bool settingIconStatus READ settingIconStatus WRITE setSettingIconStatus NOTIFY settingIconStatusChanged)
	Q_PROPERTY(QVector<int> inspecVector READ inspecVector WRITE setInspecVector NOTIFY inspecVectorChanged)


	Q_PROPERTY(bool powerEnabled READ powerEnabled WRITE setPowerEnabled NOTIFY powerEnabledChanged)
	Q_PROPERTY(bool stepEnabled READ stepEnabled WRITE setStepEnabled NOTIFY stepEnabledChanged)
	Q_PROPERTY(bool homeEnabled READ homeEnabled WRITE setHomeEnabled NOTIFY homeEnabledChanged)
public:
	explicit ControlBarVM(MasterApp* masterApp, QObject* parent = nullptr);
	~ControlBarVM();

public:
	bool playIconStatus();
	bool loginIconStatus();
	bool settingIconStatus();

	// end

private:
	MasterApp* master_app;
	rviz_visual_tools::RemoteReciever remote_reciever_;

	bool _powerEnabled;
	bool _stepEnabled;
	bool _homeEnabled;
	QVector<int> _inspecVector = { 0, 0, 0, 0, 0};

	bool _playIconStatus;
	bool _loginIconStatus;
	bool _settingIconStatus;
	// end

	bool _isComponentCompleted;

Q_SIGNALS:
	void playIconStatusChanged();
	void setPlayIconIsClicked(bool value);
	void loginIconStatusChanged();
	void settingIconStatusChanged();
	void inspecVectorChanged();

	void powerEnabledChanged();
	void stepEnabledChanged();
	void homeEnabledChanged();
	// end
	void setCurrentIndexTab(int index);
	void setConfigTomOView();
	void setEnableButton(QString nameButton, bool isEnable);

public Q_SLOTS:
	void setPlayIconStatus(bool);
	void setLoginIconStatus(bool);
	void setSettingIconStatus(bool);

	bool powerEnabled();
	void setPowerEnabled(bool value);

	bool stepEnabled();
	void setStepEnabled(bool value);

	bool homeEnabled();
	void setHomeEnabled(bool value);

	QVector<int> inspecVector();
	void setInspecVector(QVector<int>);
	// end

	void playBtnClick(bool);
	// void home_btn_click();
	void step_btn_click();
	void state_btn_click();
	// void login_btn_click();
	// void setting_btn_click();
	void powerOnBtnClick(bool powerValue);
	void tabCurrentChanged(int);
	void setIndexTab(int);
	void setComponentCompleted();
	void setAfterUiCreated();
	void checkExitPass(QString);
	int checkChangePass(QString, QString, QString);
	void resetExitPass();
};

#endif	// CONTROLBARVM_H
