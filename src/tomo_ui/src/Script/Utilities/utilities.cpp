#include "utilities.h"
#include <tomo_utils/tomo_utils.h>
#include <ros/ros.h>
#include <boost/algorithm/string.hpp>

bool checkPathExists(const std::string& name)
{
	struct stat buffer;
	return (stat(name.c_str(), &buffer) == 0);
}

std::string convertToStdString(QString qstring)
{
	return qstring.toStdString();
}
QString convertToQString(std::string string)
{
	return QString::fromStdString(string);
}

std::string getCurrentDay()
{
	return getCurrentTime(true).substr(0, 10);
}
std::string getCurrentHour()
{
	return getCurrentTime(true).substr(11, 2);
}

void UI_INFO( const char* format, ...)
{
    va_list args;
    va_start(args, format);

    char buffer[256];  // Adjust the buffer size as needed
    vsnprintf(buffer, sizeof(buffer), format, args);

    va_end(args);

   	ROS_INFO("%s", buffer);
	std::string myString(buffer);
	std::string infoContent = "<span style=color:#ffffff> [INFO] [" + std::to_string(getCurrentSec())  + "]: "+ myString + "</span>";
	saveUILog(infoContent);
}

void UI_WARN( const char* format, ...)
{
    va_list args;
    va_start(args, format);

    char buffer[256];  // Adjust the buffer size as needed
    vsnprintf(buffer, sizeof(buffer), format, args);

    va_end(args);

   	ROS_WARN("%s", buffer);
	std::string myString(buffer);
	std::string warnContent = "<span style=color:#FFC706> [WARN] [" + std::to_string(getCurrentSec()) + "]: " + myString + "</span>";
	saveUILog(warnContent);
}

void UI_ERROR( const char* format, ...)
{
    va_list args;
    va_start(args, format);

    char buffer[256];  // Adjust the buffer size as needed
    vsnprintf(buffer, sizeof(buffer), format, args);

    va_end(args);

   	ROS_ERROR("%s", buffer);
	std::string myString(buffer);
	std::string errorContent = "<span style=color:#EF190F> [ERROR] [" + std::to_string(getCurrentSec()) + "]: " + myString + "</span>";
	saveUILog(errorContent);
}

std::string tempCurrentDay;
std::string tempCurrentHour;
std::string tempUiLogPath				 = getenv("HOME") + std::string("/tomo_stats/ui_log/");
std::string log_style = "<body style='font-family:Monospace; font-size:16px; color:white; background-color:black;'>\n";
std::shared_ptr<std::ofstream> uiLogFile = std::make_shared<std::ofstream>();
std::string logFilePath;
void saveUILog(std::string logLine)
{
	logUIReset();
	*uiLogFile << logLine << "<br>";
	*uiLogFile << std::flush;
}
void logUIReset()
{
	if(!checkPathExists(tempUiLogPath)){
		mkdir(tempUiLogPath.c_str(), 0777);
		tempCurrentHour = "";
	}
	if(!checkPathExists(logFilePath)){
		tempCurrentHour = "";
	}
	if(tempCurrentHour != getCurrentHour()) {
		tempCurrentHour = getCurrentHour();
		closeUIFile();
		std::string curTime = getCurrentTime(true);
		boost::replace_all(curTime, ":", ".");
		logFilePath = tempUiLogPath + curTime + ".htm";
		uiLogFile->open(logFilePath);
		*uiLogFile << log_style;
	}
}

void closeUIFile()
{
	if(uiLogFile->is_open())
		uiLogFile->close();
}