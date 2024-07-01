#include "login_vm.h"
#include <ros/ros.h>

// My include
#include "../Script/Config/cfg_app.h"
#include "../Script/Utilities/utilities.h"
// end

LoginVM::LoginVM(MasterApp* master_app, QObject* parent) : master_app(master_app), QObject{parent}
{
	_username		  = "";
	_level			  = "";
	_lenghtPassword	  = 0;
	_rememberUsername = "";
	_numOfAcc		  = 0;
	allDataAccount	  = {};
	mAccount		  = {};

	std::string configPath		   = getenv("HOME") + std::string("/tomo_config/account.yaml");
	fileSettingAccount			   = QString::fromStdString(configPath);
	_firstCreateFileSettingAccount = false;
	_flagOvercheckPassword		   = false;
	readFileSettingAccount();
	setPermission();

	timer = new QTimer();
	timer->setInterval(900000);
	connect(timer, &QTimer::timeout, this, &LoginVM::appNowInactive);
	connect(timer, &QTimer::timeout, timer, &QTimer::stop);

	if(master_app->isSim())
		setSuperUserActive(true);
}

LoginVM::~LoginVM()
{
	delete timer;
}

bool LoginVM::setItemAt(int index, const AccountFeature& item)
{
	if(index < 0 || index >= mAccount.size())
		return false;
	const AccountFeature& oldItem = mAccount.at(index);
	if(item.user == oldItem.user && item.level == oldItem.level && item.check == oldItem.check)
		return false;
	mAccount[index] = item;
	return true;
}

QVector<AccountFeature> LoginVM::items() const
{
	return mAccount;
}

bool retry = false;
void LoginVM::readFileSettingAccount()
{
	// Check file exists and file is empty
	if(!QFile::exists(fileSettingAccount) || retry) {
		_firstCreateFileSettingAccount = true;
		_numOfAcc					   = 1;
		QHash<QString, QString> tempData;

		// If the file is new, create a default account
		tempData["username"]	   = "Admin";
		QString defaultPass		   = QString::fromUtf8(QCryptographicHash::hash("1", QCryptographicHash::Sha3_256));
		QByteArray utf8Bytes	   = defaultPass.toUtf8();
		QString base64Password	   = QString::fromLatin1(utf8Bytes.toBase64());
		tempData["password"]	   = base64Password;
		tempData["level"]		   = "Administrator";
		tempData["lengthPassword"] = "1";
		tempData["remember"]	   = "false";
		allDataAccount.append(tempData);
		mAccount.append({QStringLiteral("Admin"), QStringLiteral("Admin"), false});
		writeFileSettingAccount();
		return;
	}

	allDataAccount.clear();
	_numOfAcc = 0;

	QFile file(fileSettingAccount);
	if(file.open(QIODevice::ReadOnly | QIODevice::Text)) {
		// Read the YAML data from the file
		QString yamlData = file.readAll();
		file.close();

		try {
			YAML::Node root = YAML::Load(yamlData.toStdString());

			// Parse the YAML data
			_numOfAcc = root["numberOfAccount"].as<int>();
			for(int i = 0; i < _numOfAcc; i++) {
				std::string accountKey = "Account_" + std::to_string(i);
				if(root[accountKey]) {
					YAML::Node accountNode = root[accountKey];
					QHash<QString, QString> tempData;
					tempData.insert("username", QString::fromStdString(accountNode["username"].as<std::string>()));
					tempData.insert("password", QString::fromStdString(accountNode["password"].as<std::string>()));
					tempData.insert("level", QString::fromStdString(accountNode["level"].as<std::string>()));
					tempData.insert("lengthPassword", QString::fromStdString(accountNode["lengthPassword"].as<std::string>()));
					tempData.insert("remember", QString::fromStdString(accountNode["remember"].as<std::string>()));
					allDataAccount.append(tempData);
				}
			}
		}
		catch(const YAML::Exception& e) {
			retry = true;
			readFileSettingAccount();
			UI_ERROR("Fail to read user data");
		}
	}
}

void LoginVM::writeFileSettingAccount()
{
	YAML::Node root;

	// Insert the number of accounts
	root["numberOfAccount"] = _numOfAcc;

	for(int i = 0; i < _numOfAcc; i++) {
		std::string accountKey = "Account_" + std::to_string(i);
		YAML::Node accountNode;

		// Retrieve data from allDataAccount
		QHash<QString, QString> tempData = allDataAccount.at(i);
		accountNode["username"]			 = tempData["username"].toStdString();
		accountNode["password"]			 = tempData["password"].toStdString();
		accountNode["level"]			 = tempData["level"].toStdString();
		accountNode["lengthPassword"]	 = tempData["lengthPassword"].toStdString();
		accountNode["remember"]			 = tempData["remember"].toStdString();

		root[accountKey] = accountNode;
	}

	try {
		// Write the YAML data to the file
		std::ofstream file(fileSettingAccount.toStdString());
		file << root;
	}
	catch(const YAML::Exception& e) {
		// Handle exceptions if necessary
	}
}

void LoginVM::prepareRememberAccount()
{
	QHash<QString, QString> tempData;
	for(int i = 0; i < allDataAccount.size(); i++) {
		tempData = QHash<QString, QString>();
		tempData = allDataAccount.at(i);
		if(tempData["remember"] == "true") {

			tempData["remember"] = "false";
			allDataAccount.replace(i, tempData);
			writeFileSettingAccount();
			//             << tempData["username"] << " - delete remember";
		}
		else {
			;
		}
	}
	return;
}

void LoginVM::readRememberAccount()
{
	QHash<QString, QString> tempData;
	for(int i = 0; i < allDataAccount.size(); i++) {
		tempData = QHash<QString, QString>();
		tempData = allDataAccount.at(i);

		if(tempData["remember"] == "true") {
			_flagOvercheckPassword = true;
			_rememberUsername	   = tempData["username"];
			_lenghtPassword		   = tempData["lenghtPassword"].toInt();
			Q_EMIT rememberAccountCheck(_rememberUsername, _lenghtPassword);
			return;
		}
		else {
			_flagOvercheckPassword = false;
			_rememberUsername	   = "";
			_lenghtPassword		   = 0;
		}
	}
	return;
}

void LoginVM::checkUserLoginInDatabase(QString user, QString pass, bool remember)
{
	// clear account remember before new account login
	if(remember)
		prepareRememberAccount();
	else {
		;
	}
	readFileSettingAccount();
	QHash<QString, QString> tempData;
	for(int i = 0; i < allDataAccount.size(); i++) {
		tempData = QHash<QString, QString>();
		tempData = allDataAccount.at(i);
		if(tempData["username"] == user) {
			Q_EMIT currentPasswordCheck(pass);

			// cryptographic password
			//            QByteArray tempPass = QCryptographicHash::hash(pass.toUtf8(), QCryptographicHash::Sha3_256);
			//            pass				= QString::fromUtf8(tempPass);
			QString tempPass	 = QString::fromUtf8(QCryptographicHash::hash(pass.toUtf8(), QCryptographicHash::Sha3_256));
			QByteArray utf8Bytes = tempPass.toUtf8();
			pass				 = QString::fromLatin1(utf8Bytes.toBase64());
			// end code
			if((tempData["password"] == pass) || _flagOvercheckPassword) {

				_username	 = tempData["username"];
				_level		 = tempData["level"];
				currentLevel = NameToLevelSignal[_level.toStdString()];
				Q_EMIT userDataBase(_username, _level);
				Q_EMIT levelChanged(true);
				levelChanged();
				setPermission();
				if(remember) {
					tempData["remember"] = "true";
					//                     << tempData["username"] << " - remember";
					allDataAccount.replace(i, tempData);
					writeFileSettingAccount();
				}
				else {
					tempData["remember"] = "false";
					//                     << tempData["username"] << " - delete remember";
					allDataAccount.replace(i, tempData);
					writeFileSettingAccount();
				}
				Q_EMIT loginCheck(0);  // login success
				UI_WARN("Login successfully. Username: %s", convertToStdString(_username).c_str());
				if(_checkFirstTime){
					_checkFirstTime = false;
					master_app->xaviersTimeCheck();
				}
				updateIdleTimer();
				return;
			}
			else {
				Q_EMIT loginCheck(1);  // password error
				UI_WARN("Login failed, password wrong. Username: %s", convertToStdString(user).c_str());
				return;
			}
		}
		else {
			;
		}
	}
	Q_EMIT loginCheck(2);  // user does not exits or error!
	UI_WARN("Login failed, user does not exits. Username: %s", convertToStdString(user).c_str());
	return;
}

void LoginVM::checkPasswordChangeInDatabase(QString pass)
{
	readFileSettingAccount();
	QHash<QString, QString> tempData;
	for(int i = 0; i < allDataAccount.size(); i++) {
		tempData = QHash<QString, QString>();
		tempData = allDataAccount.at(i);
		if(tempData["username"] == _username) {
			if(!isPasswordValid(pass))	//
			{
				UI_WARN("Change password failed, new password invalid. Username: %s", convertToStdString(_username).c_str());
				Q_EMIT changePasswordCheck(2);	// new password invalid
				return;
			}
			QString passWord = pass;
			// cryptographic password
			QString tempPass	 = QString::fromUtf8(QCryptographicHash::hash(pass.toUtf8(), QCryptographicHash::Sha3_256));
			QByteArray utf8Bytes = tempPass.toUtf8();
			pass				 = QString::fromLatin1(utf8Bytes.toBase64());
			// end code

			if(tempData["password"] == pass) {
				UI_WARN("Change password failed, new password same before. Username: %s", convertToStdString(_username).c_str());
				Q_EMIT changePasswordCheck(1);	// password same before
				return;
			}
			else {
				Q_EMIT currentPasswordCheck(passWord);
				tempData["password"] = pass;
				allDataAccount.replace(i, tempData);
				writeFileSettingAccount();
				UI_WARN("Change successfully. Username: %s", convertToStdString(_username).c_str());
				Q_EMIT changePasswordCheck(0);	// change password success
				return;
			}
		}
		else {
			;
		}
	}
	UI_WARN("Change password failed, new password same before. Username: %s", convertToStdString(_username).c_str());
	Q_EMIT changePasswordCheck(2);	// lost data of user
	return;
}

bool LoginVM::isPasswordValid(QString pw)
{
	QRegularExpression regex("^(?=.*[a-z])(?=.*[A-Z]).{8,}$");
	QRegularExpressionMatch match = regex.match(pw);
	return match.hasMatch();
}

void LoginVM::checkAccountCreateInDatabase(QString user, QString pass, QString level)
{
	// convert level string to int
	QHash<QString, int> accessLevelPriority;
	accessLevelPriority.insert("Administrator", 0);
	accessLevelPriority.insert("Technician", 1);
	accessLevelPriority.insert("Operator", 2);

	readFileSettingAccount();
	QHash<QString, QString> tempData;
	int numberCreatedAcc[3] = {0, 0, 0};

	for(int i = 0; i < allDataAccount.size(); i++) {
		tempData = QHash<QString, QString>();
		tempData = allDataAccount.at(i);
		if(tempData["level"] == "Administrator") {
			numberCreatedAcc[0]++;
		}
		else if(tempData["level"] == "Technician") {
			numberCreatedAcc[1]++;
		}
		else if(tempData["level"] == "Operator") {
			numberCreatedAcc[2]++;
		}
	}
	for(int i = 0; i < allDataAccount.size(); i++) {
		tempData = QHash<QString, QString>();
		tempData = allDataAccount.at(i);

		if(tempData["username"] == user) {
			Q_EMIT createNewAccountCheck(1);  // user already have in database
			return;
		}
		else if(accessLevelPriority.value(level) < accessLevelPriority.value(_level)) {
			Q_EMIT createNewAccountCheck(2);  // user can not create account have level higher than current account login
			return;
		}
		else if(!isPasswordValid(pass)) {
			Q_EMIT createNewAccountCheck(3);  // password invalid
			return;
		}
		else if((accessLevelPriority.value(level) == 0) && (numberCreatedAcc[0] >= 1)) {
			Q_EMIT createNewAccountCheck(4);  // exceed max number of Administrator acc
			return;
		}
		else if((accessLevelPriority.value(level) == 1) && (numberCreatedAcc[1] >= 4)) {
			Q_EMIT createNewAccountCheck(5);  // exceed max number of Technician acc
			return;
		}
		else if((accessLevelPriority.value(level) == 2) && (numberCreatedAcc[2] >= 10)) {
			Q_EMIT createNewAccountCheck(6);  // exceed max number of Operator acc
			return;
		}
		else {
			;
		}
	}
	int tempLenght = pass.length();
	// cryptographic password
	QString tempPass	 = QString::fromUtf8(QCryptographicHash::hash(pass.toUtf8(), QCryptographicHash::Sha3_256));
	QByteArray utf8Bytes = tempPass.toUtf8();
	pass				 = QString::fromLatin1(utf8Bytes.toBase64());
	// end code

	_numOfAcc++;
	tempData				   = QHash<QString, QString>();
	tempData["username"]	   = user;
	tempData["password"]	   = pass;
	tempData["level"]		   = level;
	tempData["lenghtPassword"] = QString::number(tempLenght);
	tempData["remember"]	   = "false";

	allDataAccount.append(tempData);
	Q_EMIT preAppendItem();
	mAccount.append({user, level, false});
	Q_EMIT postAppendItem();
	writeFileSettingAccount();
	Q_EMIT createNewAccountCheck(0);  // create new user success
	return;
}

void LoginVM::checkDeleteAccountInDatabase()
{
	// convert level string to int
	QHash<QString, int> accessLevelPriority;
	accessLevelPriority.insert("Administrator", 0);
	accessLevelPriority.insert("Technician", 1);
	accessLevelPriority.insert("Operator", 2);

	try {
		readFileSettingAccount();

		// create list index account delete from ui
		QVector<int> indexCheckQ_ui		= {};
		std::vector<int> indexCheckC_ui = {};

		for(int j = 0; j < mAccount.size(); j++) {
			if(mAccount.at(j).check) {
				indexCheckQ_ui.append(j);
			}
			else {
				;
			}
		}
		if(indexCheckQ_ui.isEmpty()) {
			Q_EMIT deleteAccountCheck(2);  // No account delete
			return;
		}
		else {
			;
		}

		// sort index decrease
#if QT_VERSION <= QT_VERSION_CHECK(5, 14, 0)
		indexCheckC_ui = indexCheckQ_ui.toStdVector();
		indexCheckQ_ui.clear();
		std::reverse(indexCheckC_ui.begin(), indexCheckC_ui.end());
		indexCheckQ_ui = QVector<int>::fromStdVector(indexCheckC_ui);
#else
		indexCheckC_ui = std::vector<int>(indexCheckQ_ui.begin(), indexCheckQ_ui.end());
		indexCheckQ_ui.clear();
		std::reverse(indexCheckC_ui.begin(), indexCheckC_ui.end());
		indexCheckQ_ui = QVector<int>(indexCheckC_ui.begin(), indexCheckC_ui.end());
#endif

		// create list index account delete from database
		QHash<QString, QString> tempData;

		QVector<int> indexCheckQ_dt		= {};
		std::vector<int> indexCheckC_dt = {};

		for(int i = 0; i < allDataAccount.size(); i++) {
			tempData = QHash<QString, QString>();
			tempData = allDataAccount.at(i);
			for(int j = 0; j < indexCheckQ_ui.size(); j++) {
				if(tempData["username"] == mAccount.at(indexCheckQ_ui.at(j)).user) {
					int level_indata   = accessLevelPriority.value(tempData["level"]);
					int level_indelete = accessLevelPriority.value(mAccount.at(indexCheckQ_ui.at(j)).level);
					if(level_indelete <= level_indata) {
						indexCheckQ_dt.append(i);
					}
					else {
						Q_EMIT deleteAccountCheck(1);  // don't have a permission delete
						return;
					}
				}
				else {
					;
				}
			}
		}

		// sort index increase
#if QT_VERSION <= QT_VERSION_CHECK(5, 14, 0)
		indexCheckC_dt = indexCheckQ_dt.toStdVector();
		indexCheckQ_dt.clear();
		std::reverse(indexCheckC_dt.begin(), indexCheckC_dt.end());
		indexCheckQ_dt = QVector<int>::fromStdVector(indexCheckC_dt);
#else
		indexCheckC_dt = std::vector<int>(indexCheckQ_dt.begin(), indexCheckQ_dt.end());
		indexCheckQ_dt.clear();
		std::reverse(indexCheckC_dt.begin(), indexCheckC_dt.end());
		indexCheckQ_dt = QVector<int>(indexCheckC_dt.begin(), indexCheckC_dt.end());
#endif
		// do delete
		for(int i = 0; i < indexCheckQ_dt.size(); i++) {
			allDataAccount.removeAt(indexCheckQ_dt.at(i));
			_numOfAcc--;
		}
		for(int i = 0; i < indexCheckQ_ui.size(); i++) {
			Q_EMIT preRemoveItem(indexCheckQ_ui.at(i));
			mAccount.removeAt(indexCheckQ_ui.at(i));
			Q_EMIT postRemoveItem();
		}

		writeFileSettingAccount();
		Q_EMIT deleteAccountCheck(0);  // delete success
		return;
	}
	catch(...) {
		Q_EMIT deleteAccountCheck(3);  // lost data of user
		return;
	}
}

void LoginVM::clearDataCurrentAccount()
{
	_username			   = "";
	_level				   = "";
	levelChanged();
	_lenghtPassword		   = 0;
	_rememberUsername	   = "";
	_flagOvercheckPassword = false;
	Q_EMIT levelChanged(false);
	currentLevel = LEVELCOUNT;
	setPermission();
	updateIdleTimer();
}

void LoginVM::autoLogout()
{
	if(_username != ""){
		Q_EMIT autoLogoutCheck();
		UI_WARN("Auto Logout Triggered");
	}
}

void LoginVM::loadListAccount()
{
	if(_firstCreateFileSettingAccount) {
		;
	}
	else {
		QHash<QString, QString> tempData;
		for(int i = 0; i < allDataAccount.size(); i++) {
			tempData = QHash<QString, QString>();
			tempData = allDataAccount.at(i);
			mAccount.append({tempData["username"], tempData["level"], false});
		}
	}
	return;
}

bool LoginVM::flagOvercheckPassword()
{
	return _flagOvercheckPassword;
}

void LoginVM::setflagOvercheckPassword(bool overpass)
{
	_flagOvercheckPassword = overpass;
}

void LoginVM::setPermission()
{
	setControlEnabled(currentLevel <= master_app->config.controlMasterLevel);
	setProductionEnabled(currentLevel <= master_app->config.productionLevel);
	setIoControlEnabled(currentLevel <= master_app->config.ioControlLevel);
	setSettingEnabled(currentLevel <= master_app->config.settingLevel);
	setSystemLogEnabled(currentLevel <= master_app->config.systemLogLevel);
	setParametersEnabled(currentLevel <= master_app->config.parametersLevel);
	setExitPassEnabled(currentLevel <= master_app->config.exitPassLevel);
}

bool LoginVM::accepted()
{
	return _accepted;
}

void LoginVM::setAccepted(bool value)
{
	_accepted = value;
	Q_EMIT acceptedChanged();
}
bool LoginVM::checkEasterEgg(QString process)
{
	easterEgg.push_back(convertToStdString(process));

	for(int i = 0; i < easterEgg.size(); i++) {
		if(easterEgg[i] != superUserCondition[i]) {
			_accepted = false;
			easterEgg.clear();
			break;
		}
		else if(i == 3 && easterEgg == superUserCondition) {
			_accepted = true;
			easterEgg.clear();
		}
	}
}
void LoginVM::checkSuperUser(QString password)
{
	_accepted = false;
	if(password == "ETcontrol") {
		currentLevel = EMAGE;
		setPermission();
	}
	setSuperUserActive(password == "ETcontrol");
}

void LoginVM::updateIdleTimer()
{
	if(currentLevel < OPERATOR) {
		timer->start();
	}
	else {
		if(timer->isActive())
			timer->stop();
	}
}

void LoginVM::appNowInactive()
{
	autoLogout();
}
bool LoginVM::controlEnabled()
{
	return _controlEnabled;
}

bool LoginVM::productionEnabled()
{
	return _productionEnabled;
}

bool LoginVM::ioControlEnabled()
{
	return _ioControlEnabled;
}

bool LoginVM::settingEnabled()
{
	return _settingEnabled;
}

bool LoginVM::systemLogEnabled()
{
	return _systemLogEnabled;
}

bool LoginVM::parametersEnabled()
{
	return _parametersEnabled;
}

bool LoginVM::superUserActive()
{
	return _superUserActive;
}

bool LoginVM::exitPassEnabled()
{
	return _exitPassEnabled;
}

void LoginVM::setControlEnabled(bool value)
{
	if(value != _controlEnabled) {
		_controlEnabled = value;
		Q_EMIT controlEnabledChanged();
	}
}
void LoginVM::setProductionEnabled(bool value)
{
	if(value != _productionEnabled) {
		_productionEnabled = value;
		Q_EMIT productionEnabledChanged();
	}
}

void LoginVM::setIoControlEnabled(bool value)
{
	if(value != _ioControlEnabled) {
		_ioControlEnabled = value;
		Q_EMIT ioControlEnabledChanged();
	}
}

void LoginVM::setSettingEnabled(bool value)
{
	if(value != _settingEnabled) {
		_settingEnabled = value;
		Q_EMIT settingEnabledChanged();
	}
}

void LoginVM::setSystemLogEnabled(bool value)
{
	if(value != _systemLogEnabled) {
		_systemLogEnabled = value;
		Q_EMIT systemLogEnabledChanged();
	}
}

void LoginVM::setParametersEnabled(bool value)
{
	if(value != _parametersEnabled) {
		_parametersEnabled = value;
		Q_EMIT parametersEnabledChanged();
	}
}

void LoginVM::setExitPassEnabled(bool value)
{
	if(value != _exitPassEnabled) {
		_exitPassEnabled = value;
		Q_EMIT exitPassEnabledChanged();
	}
}

void LoginVM::callSetExitPassEnabled(){
	setExitPassEnabled(currentLevel <= master_app->config.exitPassLevel);
}

void LoginVM::setSuperUserActive(bool value)
{
	if(value)
		UI_WARN("Super User: Activated");
	if(value != _superUserActive) {
		_superUserActive = value;
		Q_EMIT superUserActiveChanged(value);
	}
	master_app->isSuperUser = value;
}
QString LoginVM::level(){
	return _level;
}
void LoginVM::setLevel(QString value){
	_level = value;
	levelChanged();
}
// Need to check architecture of json file
