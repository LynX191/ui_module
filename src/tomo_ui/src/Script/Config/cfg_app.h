#ifndef CFGAPP_H
#define CFGAPP_H

// My include
#include <vector>
#include <fstream>
#include "../../Model/TomoModel/mode_model.h"
#include "../../Model/TomoModel/state_model.h"
#include "../Define/struct_def.h"
// end

using namespace std;

class CfgApp
{
public:
	CfgApp();
	~CfgApp();

	static CfgApp* instance();

	StateBackupProcess stateBackupProcess;
	void readFileConfig();
	void writeFileConfig();

	QVector<QString> trackNames;
	QHash<QString, QStringList> docNames;
	int getCameraFocusFromString(std::string);
	int convertAccessLevel(std::string);
	// for focus tab view and ,maximize
	//Not show in UI
	std::vector<std::string> camsFocus		  = {"Tomo Pose", "Tomo", "Shipper", "Carton Transfer"};
	int		tabIndex		  = PRODUCTION;
	int 	tabOfSystemlog	  = 0;
	int 	heightOfSystemlog = 220;
	int 	yOfSystemlog	  = 860;
	std::string 	bgColorCmd		  = "#000000";
	int 	textSizeCmd	   = 16;
	std::string 	textFontCmd	   = "Monospace";
	int 	lineSpacingCmd = 2;
	int 	limitLineCmd   = 10000;
	int 	scrollLineCmd  = 5;
	std::string exitPass = "77+977+9P2JR77+9e++/vX5u77+9";
	int		productId	= 0;
	int 	freeSpace  = 5;
	bool 	dryrunMode = false;
	bool 	bypassMode = false;
	bool 	soundMode = true;
    int     sliderS1  = 20;
    int     sliderS2  = 20;
    int     sliderS3x = 100;
    int     sliderS3y = 100;
    int     sliderS4x = 10;
    int     sliderS4y = 10;
    int     sliderS9x = 200;
    int     sliderS9y = 200;
    int     sliderS9z = 200;


	std::vector<std::string> camerasID		  = {"18443010611D631200", "184430108165940F00", "1844301041F6621200"};
	std::vector<std::string> defaultCamerasID		  = {"18443010611D631200", "184430108165940F00", "1844301041F6621200"};
	//Access Rights
	int  	settingLevel	   = ADMINISTRATOR;
	int  	systemLogLevel	   = ADMINISTRATOR;
	int  	parametersLevel	   = ADMINISTRATOR;
	int  	controlMasterLevel = ADMINISTRATOR;
	int  	productionLevel	   = ADMINISTRATOR;
	int  	ioControlLevel	   = ADMINISTRATOR;
	int 	exitPassLevel	   = ADMINISTRATOR;
	/*---------------------------*/

	bool configValid = true;

protected:
	string fileConfig = getenv("HOME") + std::string("/tomo_config/carton_ui.yaml");
};

#endif	// CFGAPP_H
