#ifndef UTILITIES_H
#define UTILITIES_H

#pragma once
#include <QString>
#include <iostream>
#include <sys/stat.h>
#include <vector>
#include <fstream>

bool checkPathExists(const std::string& name);
std::string convertToStdString(QString qstring);
QString convertToQString(std::string string);


std::string getCurrentDay();
std::string getCurrentHour();

void UI_INFO(const char* format, ...);
void UI_WARN(const char* format, ...);
void UI_ERROR(const char* format, ...);
void saveUILog(std::string);
void logUIReset();
void closeUIFile();
#endif	// UTILITIES_H