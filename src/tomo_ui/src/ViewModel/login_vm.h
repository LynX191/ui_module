#ifndef LOGINVM_H
#define LOGINVM_H

#pragma once

#include <QByteArray>
#include <QCryptographicHash>
#include <QDir>
#include <QFile>
#include <QHash>
#include <QIODevice>
#include <QRegularExpression>
#include <QString>
#include <QTextStream>
#include <QTimer>
#include <QVector>
#include <iostream>

#include "master_app.h"
#include <fstream>
#include <yaml-cpp/yaml.h>

// My include
// end
struct AccountFeature
{
	QString user;
	QString level;
	bool check;
};

class LoginVM : public QObject
{
	Q_OBJECT
	Q_PROPERTY(bool flagOvercheckPassword READ flagOvercheckPassword WRITE setflagOvercheckPassword NOTIFY flagOvercheckPasswordChanged)
	Q_PROPERTY(bool controlEnabled READ controlEnabled WRITE setControlEnabled NOTIFY controlEnabledChanged)
	Q_PROPERTY(bool productionEnabled READ productionEnabled WRITE setProductionEnabled NOTIFY productionEnabledChanged)
	Q_PROPERTY(bool ioControlEnabled READ ioControlEnabled WRITE setIoControlEnabled NOTIFY ioControlEnabledChanged)
	Q_PROPERTY(bool settingEnabled READ settingEnabled WRITE setSettingEnabled NOTIFY settingEnabledChanged)
	Q_PROPERTY(bool systemLogEnabled READ systemLogEnabled WRITE setSystemLogEnabled NOTIFY systemLogEnabledChanged)
	Q_PROPERTY(bool parametersEnabled READ parametersEnabled WRITE setParametersEnabled NOTIFY parametersEnabledChanged)
	Q_PROPERTY(bool exitPassEnabled READ exitPassEnabled WRITE setExitPassEnabled NOTIFY exitPassEnabledChanged)

	Q_PROPERTY(bool superUserActive READ superUserActive WRITE setSuperUserActive NOTIFY superUserActiveChanged)
	Q_PROPERTY(bool accepted 		READ accepted		WRITE setAccepted NOTIFY acceptedChanged)
	Q_PROPERTY(QString level 		READ level		WRITE setLevel NOTIFY levelChanged)

public:
	explicit LoginVM(MasterApp* masterApp, QObject* parent = nullptr);
	~LoginVM();

public:
	bool setItemAt(int, const AccountFeature&);
	QVector<AccountFeature> items() const;
	bool flagOvercheckPassword();

	bool _controlEnabled;
	bool _productionEnabled;
	bool _ioControlEnabled;
	bool _settingEnabled;
	bool _systemLogEnabled;
	bool _parametersEnabled;
	bool _exitPassEnabled;

	bool _accepted	= false;
	bool _superUserActive = false;
	bool superUserActive();
	bool controlEnabled();
	bool productionEnabled();
	bool ioControlEnabled();
	bool exitPassEnabled();
	bool settingEnabled();
	bool systemLogEnabled();
	bool parametersEnabled();

private:
	QTimer* timer;
	MasterApp* master_app;
	QString _username;
	QString _level;
	int currentLevel = LEVELCOUNT;
	int _lenghtPassword;
	QString _rememberUsername;
	QVector<QHash<QString, QString>> allDataAccount;
	QVector<AccountFeature> mAccount;
	std::vector<std::string> easterEgg;
	std::vector<std::string> superUserCondition = {"Control","Server","Planner","Sequence"};
	QString fileSettingAccount;

	bool _flagOvercheckPassword;
	bool _firstCreateFileSettingAccount;
	bool _checkFirstTime = true;
	int _numOfAcc;
	void readFileSettingAccount();
	void writeFileSettingAccount();
	void prepareRememberAccount();
	bool isPasswordValid(QString pw);

Q_SIGNALS:
	void levelChanged();
	void preAppendItem();
	void postAppendItem();
	void preRemoveItem(int);
	void postRemoveItem();
	void loginCheck(int error_code);
	void changePasswordCheck(int error_code);
	void createNewAccountCheck(int error_code);
	void deleteAccountCheck(int error_code);
	void currentPasswordCheck(QString currentPassword);
	void rememberAccountCheck(QString userRemember, int lenghtPass);
	void userDataBase(QString user, QString level);
	void flagOvercheckPasswordChanged();
	void levelChanged(bool isLogin);
	void autoLogoutCheck();

	void controlEnabledChanged();
	void productionEnabledChanged();
	void ioControlEnabledChanged();
	void settingEnabledChanged();
	void systemLogEnabledChanged();
	void parametersEnabledChanged();
	void exitPassEnabledChanged();

	void superUserActiveChanged(bool active);
	void acceptedChanged();

public Q_SLOTS:
	QString level();
	void setLevel(QString);

	void checkUserLoginInDatabase(QString, QString, bool);
	void checkPasswordChangeInDatabase(QString);
	void checkAccountCreateInDatabase(QString, QString, QString);
	void checkDeleteAccountInDatabase();
	void clearDataCurrentAccount();
	void readRememberAccount();
	void loadListAccount();
	void setflagOvercheckPassword(bool);

	void setPermission();
	void setControlEnabled(bool);
	void setProductionEnabled(bool);
	void setIoControlEnabled(bool);
	void setSettingEnabled(bool);
	void setSystemLogEnabled(bool);
	void setParametersEnabled(bool);
	void setExitPassEnabled(bool);
	void setSuperUserActive(bool);
	bool checkEasterEgg(QString);
	void autoLogout();

	bool accepted();
	void setAccepted(bool value);
	void checkSuperUser(QString);
	void updateIdleTimer();
	void appNowInactive();
	void callSetExitPassEnabled();
};

#endif	// LOGINVM_H
