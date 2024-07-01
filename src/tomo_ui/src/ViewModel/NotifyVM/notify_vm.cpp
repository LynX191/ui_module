#include "notify_vm.h"
#include <QDebug>

NotifyVM::NotifyVM(MasterApp* master_app, QObject* parent) : master_app(master_app), QObject{parent}
{
	numOfNoti = 0;
	mNotify.clear();

	std::string filePath = getenv("HOME") + std::string("/tomo_stats/notify_log/notify_view.yaml");
	fileNotiBarLog		 = QString::fromStdString(filePath);

	readFileNotify();
}

QVector<NotifyFeature> NotifyVM::items() const
{
	return mNotify;
}

void NotifyVM::readFileNotify()
{
	if(!QFile::exists(fileNotiBarLog)) {
		writeFileNotify();
		return;
	}
	allNotify.clear();
	numOfNoti = 0;
	QVector<NotifyFeature> tempNotify;

	QFile file(fileNotiBarLog);
	if(file.open(QIODevice::ReadOnly | QIODevice::Text)) {
		// Read the YAML data from the file
		QString yamlData = file.readAll();
		file.close();

		try {
			YAML::Node root = YAML::Load(yamlData.toStdString());

			// Parse the YAML data
			numOfNoti = root["numberOfNotify"].as<int>();
			for(int i = 0; i < numOfNoti; i++) {
				std::string notiKey	   = "Notify_" + std::to_string(i);
				YAML::Node accountNode = root[notiKey];
				QHash<QString, QString> tempData;
				tempData.insert("type", QString::fromStdString(accountNode["type"].as<std::string>()));
				tempData.insert("messID", QString::fromStdString(accountNode["messID"].as<std::string>()));
				tempData.insert("content", QString::fromStdString(accountNode["content"].as<std::string>()));
				tempData.insert("time", QString::fromStdString(accountNode["time"].as<std::string>()));
				allNotify.append(tempData);

				NotifyFeature temp;
				temp.type	 = QString::fromStdString(accountNode["type"].as<std::string>());
				temp.messID	 = QString::fromStdString(accountNode["messID"].as<std::string>());
				temp.content = QString::fromStdString(accountNode["content"].as<std::string>());
				temp.time	 = QString::fromStdString(accountNode["time"].as<std::string>());
				tempNotify.append(temp);
			}

			mNotify = tempNotify;
		}
		catch(const YAML::Exception& e) {
			// Handle YAML parsing errors
		}
	}
}
void NotifyVM::writeFileNotify()
{
	QDir directory(logDir);
	if(!directory.exists()) {
		if(!directory.mkpath(".")) {
			qDebug() << "Failed to create directory.";
			return;
		}
	}
	YAML::Node root;
	// Insert the number of accounts
	root["numberOfNotify"] = numOfNoti;

	for(int i = 0; i < numOfNoti; i++) {
		std::string notiKey = "Notify_" + std::to_string(i);
		YAML::Node accountNode;

		// Retrieve data from allNotify
		QHash<QString, QString> tempData = allNotify.at(i);
		accountNode["type"]				 = tempData["type"].toStdString();
		accountNode["messID"]			 = tempData["messID"].toStdString();
		accountNode["content"]			 = tempData["content"].toStdString();
		accountNode["time"]				 = tempData["time"].toStdString();

		root[notiKey] = accountNode;
	}

	try {
		// Write the YAML data to the file
		std::ofstream file(fileNotiBarLog.toStdString());
		file << root;
	}
	catch(const YAML::Exception& e) {
		// Handle exceptions if necessary
	}
}
void NotifyVM::addNotifyToDatabase(QString type, QString messID, QString content, QString time)
{
	readFileNotify();
	QHash<QString, QString> tempData;

	numOfNoti++;
	tempData			= QHash<QString, QString>();
	tempData["type"]	= type;
	tempData["messID"]	= messID;
	tempData["content"] = content;
	tempData["time"]	= time;

	allNotify.prepend(tempData);
	Q_EMIT prePrependItem();
	mNotify.prepend({type, messID, content, time});
	Q_EMIT postPrependItem();
	// Check if the number of notifications exceeds 10
	if(numOfNoti > 200) {
		int excessCount = numOfNoti - 200;
		for(int i = 0; i < excessCount; ++i) {
			allNotify.removeLast();
			Q_EMIT preRemoveItem(numOfNoti - 1);
			mNotify.removeLast();
			Q_EMIT postRemoveItem();
			--numOfNoti;
		}
	}
	writeFileNotify();
	return;
}

bool NotifyVM::setItemAt(int index, const NotifyFeature& item)
{
	if(index < 0 || index >= mNotify.size())
		return false;
	const NotifyFeature& oldItem = mNotify.at(index);
	if(item.type == oldItem.type && item.messID == oldItem.messID && item.content == oldItem.content && item.time == oldItem.time)
		return false;
	mNotify[index] = item;
	return true;
}
void NotifyVM::checkDeleteNotifyInDatabase(QString messageID)
{
	try {
		readFileNotify();

		// create list index account delete from ui
		QVector<int> indexCheckQ_ui		= {};
		std::vector<int> indexCheckC_ui = {};

		for(int j = 0; j < mNotify.size(); j++) {
			if(mNotify.at(j).messID == messageID) {
				indexCheckQ_ui.append(j);
			}
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
		for(int i = 0; i < allNotify.size(); i++) {
			tempData = QHash<QString, QString>();
			tempData = allNotify.at(i);
			for(int j = 0; j < indexCheckQ_ui.size(); j++) {
				if(tempData["messID"] == mNotify.at(indexCheckQ_ui.at(j)).messID) {
					indexCheckQ_dt.append(i);
					break;
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
			allNotify.removeAt(indexCheckQ_dt.at(i));
			numOfNoti--;
		}
		for(int i = 0; i < indexCheckQ_ui.size(); i++) {
			Q_EMIT preRemoveItem(indexCheckQ_ui.at(i));

			mNotify.removeAt(indexCheckQ_ui.at(i));
			Q_EMIT postRemoveItem();
		}
		writeFileNotify();
		return;
	}
	catch(...) {
		UI_ERROR("Can not delete notify  !!!");
		return;
	}
}
