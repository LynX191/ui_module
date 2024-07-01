#include "cfg_app.h"
#include <yaml-cpp/yaml.h>

#include "../Utilities/utilities.h"
#include <QDir>
#include <QFile>
#include <QTextStream>
#include <ros/ros.h>
CfgApp* ptrAppConfigIns = 0;

namespace YAML
{
	template <> struct convert<QString>
	{
		static Node encode(const QString& rhs)
		{
			return Node(rhs.toStdString());
		}

		static bool decode(const Node& node, QString& rhs)
		{
			if(!node.IsScalar())
				return false;
			rhs = QString::fromStdString(node.as<std::string>());
			return true;
		}
	};
}  // namespace YAML

CfgApp* CfgApp::instance()
{
	if(!ptrAppConfigIns)
		ptrAppConfigIns = new CfgApp();
	return ptrAppConfigIns;
}

CfgApp::CfgApp()
{
	limitLineCmd = 10000;
	trackNames	 = QVector<QString>{QString("TomoPose"), QString("TomoEye"), QString("Shipper"), QString("PIM")};
	QHash<QString, QStringList> temp;
	temp[QString("TomoPose")] = QStringList{"TomoPose"};
	temp[QString("TomoEye")]  = QStringList{"TomO", "Filling", "Closing", "Output"};
	temp[QString("Shipper")]  = QStringList{"Shipper", "TSTIM", "BSTIM", "Afolding"};
	temp[QString("PIM")]	  = QStringList{"Carton Transfer", "Carton"};
	docNames				  = temp;
	stateBackupProcess = StateBackupProcess::IDLE;
}

CfgApp::~CfgApp()
{
	if(ptrAppConfigIns)
		delete ptrAppConfigIns;
}

void CfgApp::readFileConfig()
{
	try {
		// Check file exits and file empyt
		fileConfig = getenv("HOME") + std::string("/tomo_config/carton_ui.yaml");
		if(!checkPathExists(fileConfig)) {
			camsFocus		  = {"Tomo Pose", "Tomo", "Shipper", "Carton Transfer"};
			tabIndex		  = PRODUCTION;
			tabOfSystemlog	  = 0;
			heightOfSystemlog = 220;
			yOfSystemlog	  = 860;
			bgColorCmd		  = "#000000";

			textSizeCmd	   = 16;
			textFontCmd	   = "Monospace";
			lineSpacingCmd = 2;
			limitLineCmd   = 10000;
			scrollLineCmd  = 5;
			productId	   = 0;
			freeSpace	   = 5;
			exitPass	   = "Admin";
			dryrunMode = false;
			bypassMode = false;
			soundMode  = true;

			sliderS1  = 20;
            sliderS2  = 20;
            sliderS3x = 100;
            sliderS3y = 100;
            sliderS4x = 10;
            sliderS4y = 10;
            sliderS9x = 200;
            sliderS9y = 200;
            sliderS9z = 200;

			settingLevel	   = ADMINISTRATOR;
			systemLogLevel	   = ADMINISTRATOR;
			parametersLevel	   = ADMINISTRATOR;			
			controlMasterLevel = ADMINISTRATOR;
			productionLevel	   = ADMINISTRATOR;
			ioControlLevel	   = ADMINISTRATOR;
			exitPassLevel	   = ADMINISTRATOR;

			camerasID		  = {"18443010611D631200", "184430108165940F00", "1844301041F6621200"};
			defaultCamerasID  = {"18443010611D631200", "184430108165940F00", "1844301041F6621200"};

			writeFileConfig();
			UI_ERROR("[CfgApp::readFileConfig] File %s don't exist", fileConfig.c_str());
			configValid = false;
		}
		else {
			try {
				YAML::Node rootObj = YAML::LoadFile(fileConfig);

				try {
					// General section
					if(!rootObj["General"] || !rootObj["CamsFocus"] || !rootObj["ControlConfig"] || !rootObj["Cameras ID"] || !rootObj["Default Cameras ID"]) {
						// Catch any other unhandled exceptions
						UI_ERROR("[CfgApp::readFileConfig[0]] Fail to read file %s ", fileConfig.c_str());
						configValid = false;

					}
						camsFocus = rootObj["CamsFocus"].as<std::vector<std::string>>();
						heightOfSystemlog = rootObj["General"]["HeightOfSystemlog"].as<int>();
						yOfSystemlog	  = rootObj["General"]["YOfSystemlog"].as<int>();
						tabOfSystemlog	  = rootObj["General"]["TabOfSystemlog"].as<int>();

						bgColorCmd	   = rootObj["General"]["BgColorCmd"].as<std::string>();
						tabIndex 	   = rootObj["General"]["TabIndex"].as<int>();
						textFontCmd	   = rootObj["General"]["TextFontCmd"].as<std::string>();
						textSizeCmd	   = rootObj["General"]["TextSizeCmd"].as<int>();
						lineSpacingCmd = rootObj["General"]["LineSpacingCmd"].as<int>();
						scrollLineCmd  = rootObj["General"]["ScrollLineCmd"].as<int>();
						exitPass  = rootObj["General"]["ExitPass"].as<std::string>();
						productId  = rootObj["General"]["ProductId"].as<int>();						
						freeSpace  = rootObj["General"]["FreeSpace"].as<double>();


					// ControlConfig section
						dryrunMode = rootObj["ControlConfig"]["DryrunMode"].as<bool>();
						bypassMode = rootObj["ControlConfig"]["BypassMode"].as<bool>();
						soundMode = rootObj["ControlConfig"]["SoundMode"].as<bool>();
						sliderS1 = rootObj["ControlConfig"]["SliderS1"].as<int>();
						sliderS2 = rootObj["ControlConfig"]["SliderS2"].as<int>();
						sliderS3x = rootObj["ControlConfig"]["SliderS3x"].as<int>();
						sliderS3y = rootObj["ControlConfig"]["SliderS3y"].as<int>();
						sliderS4x = rootObj["ControlConfig"]["SliderS4x"].as<int>();
						sliderS4y = rootObj["ControlConfig"]["SliderS4y"].as<int>();
						sliderS9x = rootObj["ControlConfig"]["SliderS9x"].as<int>();
						sliderS9y = rootObj["ControlConfig"]["SliderS9y"].as<int>();
						sliderS9z = rootObj["ControlConfig"]["SliderS9z"].as<int>();

					//Camera ID
						camerasID = rootObj["Cameras ID"].as<std::vector<std::string>>();
						defaultCamerasID = rootObj["Default Cameras ID"].as<std::vector<std::string>>();

					// AccessRights section
						settingLevel	   = rootObj["AccessRights"]["Settings"].as<int>();
						systemLogLevel	   = rootObj["AccessRights"]["System Log"].as<int>();
						parametersLevel	   = rootObj["AccessRights"]["Parameters"].as<int>();
						controlMasterLevel = rootObj["AccessRights"]["Control Master"].as<int>();
						productionLevel	   = rootObj["AccessRights"]["Production & Control"].as<int>();
						ioControlLevel	   = rootObj["AccessRights"]["IO Control"].as<int>();
						exitPassLevel	   = rootObj["AccessRights"]["ExitPass"].as<int>();

				}
				catch(...) {
					// Catch any other unhandled exceptions
					UI_ERROR("[CfgApp::readFileConfig[2]] Fail to read file %s ", fileConfig.c_str());
					configValid = false;
				}
				// Limit value
				if(camsFocus.empty())
					camsFocus = {"Tomo Pose", "Tomo", "Shipper", "PIM"};
				if(tabIndex < IO || tabIndex > PRODUCTION)
					tabIndex = PRODUCTION;
				if(tabOfSystemlog < 0 || tabOfSystemlog > 10)
					tabOfSystemlog = 0;
				if(yOfSystemlog < 0 || yOfSystemlog > 1080)
					yOfSystemlog = 860;
				if(heightOfSystemlog < 0 || heightOfSystemlog > 1080)
					heightOfSystemlog = 220;
				if(bgColorCmd.empty())
					bgColorCmd = "#000000";
				if(textSizeCmd < 10 || textSizeCmd > 48)
					textSizeCmd = 16;
				if(textFontCmd.empty())
					textFontCmd = "Monospace";
				if(lineSpacingCmd < 0 || lineSpacingCmd > 20)
					lineSpacingCmd = 2;
				if(scrollLineCmd < 0 || scrollLineCmd > 20)
					scrollLineCmd = 5;
				if(exitPass.empty())
					exitPass = "77+977+9P2JR77+9e++/vX5u77+9";
				if(productId < 0 || productId > 20)
					productId = 0;
				if(freeSpace < 5 || freeSpace > 200)
					freeSpace = 5;

				if(dryrunMode != true && dryrunMode != false)
					dryrunMode = false;
				if(bypassMode != true && bypassMode != false)
					bypassMode = true;
				if(soundMode != true && soundMode != false)
					soundMode = false;
				if(sliderS1 < 20 || sliderS1 > 3000)
					sliderS1 = 20;
				if(sliderS2 < 20 || sliderS2 > 3000)
					sliderS2 = 20;
				if(sliderS3x < 100 || sliderS3x > 1500)
					sliderS3x = 100;
				if(sliderS3y < 100 || sliderS3y > 1500)
					sliderS3y = 100;
				if(sliderS4x < 10 || sliderS4x > 3000)
					sliderS4x = 10;
				if(sliderS4y < 10 || sliderS4y > 3000)
					sliderS4y = 10;
				if(sliderS9x < 200 || sliderS9x > 500)
					sliderS9x = 200;
				if(sliderS9y < 200 || sliderS9y > 500)
					sliderS9y = 200;
				if(sliderS9z < 200 || sliderS9z > 500)
					sliderS9z = 200;

				if(camerasID.empty())
					camerasID = {"18443010611D631200", "184430108165940F00", "1844301041F6621200"};
				if(defaultCamerasID.empty())
					defaultCamerasID = {"18443010611D631200", "184430108165940F00", "1844301041F6621200"};

				if(settingLevel < ADMINISTRATOR || settingLevel > OPERATOR)
					settingLevel = ADMINISTRATOR;
				if(systemLogLevel < ADMINISTRATOR || systemLogLevel > OPERATOR)
					systemLogLevel = ADMINISTRATOR;
				if(parametersLevel < ADMINISTRATOR || parametersLevel > OPERATOR)
					parametersLevel = ADMINISTRATOR;
				if(controlMasterLevel < ADMINISTRATOR || controlMasterLevel > OPERATOR)
					controlMasterLevel = ADMINISTRATOR;
				if(productionLevel < ADMINISTRATOR || productionLevel > OPERATOR)
					productionLevel = ADMINISTRATOR;
				if(ioControlLevel < ADMINISTRATOR || ioControlLevel > OPERATOR)
					ioControlLevel = ADMINISTRATOR;
				if(exitPassLevel < ADMINISTRATOR || exitPassLevel > OPERATOR)
					exitPassLevel = ADMINISTRATOR;
			}
			catch(...) {
				// Catch any other unhandled exceptions
				UI_ERROR("[CfgApp::readFileConfig[3]] Fail to read file %s ", fileConfig.c_str());
				configValid = false;
			}
		}
	}
	catch(...) {
		// Catch any other unhandled exceptions
		UI_ERROR("[CfgApp::readFileConfig[4]] Fail to read file %s ", fileConfig.c_str());
		configValid = false;
	}
}

void CfgApp::writeFileConfig()
{
	// Create an emitter for YAML
	YAML::Emitter emitter;
	emitter << YAML::BeginMap;

	// Not show in UI
	//  CamsFocus section
	emitter << YAML::Key << "CamsFocus" << YAML::Value << YAML::BeginSeq;
	for(const auto& focus : camsFocus) {
		emitter << focus;
	}
	emitter << YAML::EndSeq;

	// General section
	emitter << YAML::Key << "General" << YAML::Value << YAML::BeginMap;
	emitter << YAML::Key << "BgColorCmd" << YAML::Value << bgColorCmd;
	emitter << YAML::Key << "TabIndex" << YAML::Value << tabIndex;					  // Not showing
	emitter << YAML::Key << "TabOfSystemlog" << YAML::Value << tabOfSystemlog;		  // Not showing
	emitter << YAML::Key << "HeightOfSystemlog" << YAML::Value << heightOfSystemlog;  // Not showing
	emitter << YAML::Key << "YOfSystemlog" << YAML::Value << yOfSystemlog;			  // Not showing
	emitter << YAML::Key << "TextSizeCmd" << YAML::Value << textSizeCmd;
	emitter << YAML::Key << "TextFontCmd" << YAML::Value << textFontCmd;
	emitter << YAML::Key << "LineSpacingCmd" << YAML::Value << lineSpacingCmd;
	emitter << YAML::Key << "LimitLineCmd" << YAML::Value << limitLineCmd;
	emitter << YAML::Key << "ScrollLineCmd" << YAML::Value << scrollLineCmd;
	emitter << YAML::Key << "ExitPass" << YAML::Value << exitPass;
	emitter << YAML::Key << "ProductId" << YAML::Value << productId;
	emitter << YAML::Key << "FreeSpace" << YAML::Value << freeSpace;
	emitter << YAML::EndMap;

	// ControlConfig section
	emitter << YAML::Key << "ControlConfig" << YAML::Value << YAML::BeginMap;

    emitter << YAML::Key << "DryrunMode" << YAML::Value << dryrunMode;
    emitter << YAML::Key << "BypassMode" << YAML::Value << bypassMode;
    emitter << YAML::Key << "SoundMode" << YAML::Value << soundMode;
    emitter << YAML::Key << "SliderS1" << YAML::Value << sliderS1;
    emitter << YAML::Key << "SliderS2" << YAML::Value << sliderS2;
    emitter << YAML::Key << "SliderS3x" << YAML::Value << sliderS3x;
    emitter << YAML::Key << "SliderS3y" << YAML::Value << sliderS3y;
    emitter << YAML::Key << "SliderS4x" << YAML::Value << sliderS4x;
    emitter << YAML::Key << "SliderS4y" << YAML::Value << sliderS4y;
    emitter << YAML::Key << "SliderS9x" << YAML::Value << sliderS9x;
    emitter << YAML::Key << "SliderS9y" << YAML::Value << sliderS9y;
    emitter << YAML::Key << "SliderS9z" << YAML::Value << sliderS9z;
	emitter << YAML::EndMap;

	//  CamsFocus section
	emitter << YAML::Key << "Cameras ID" << YAML::Value << YAML::BeginSeq;
	for(const auto& camID : camerasID) {
		emitter << camID;
	}
	emitter << YAML::EndSeq;

	emitter << YAML::Key << "Default Cameras ID" << YAML::Value << YAML::BeginSeq;
	for(const auto& camID : defaultCamerasID) {
		emitter << camID;
	}
	emitter << YAML::EndSeq;

	// AccessRights section
	emitter << YAML::Key << "AccessRights" << YAML::Value << YAML::BeginMap;
	emitter << YAML::Key << "Control Master" << YAML::Value << controlMasterLevel;
	emitter << YAML::Key << "Production & Control" << YAML::Value << productionLevel;
	emitter << YAML::Key << "IO Control" << YAML::Value << ioControlLevel;
	emitter << YAML::Key << "Settings" << YAML::Value << settingLevel;
	emitter << YAML::Key << "System Log" << YAML::Value << systemLogLevel;
	emitter << YAML::Key << "Parameters" << YAML::Value << parametersLevel;
	emitter << YAML::Key << "ExitPass" << YAML::Value << exitPassLevel;
	emitter << YAML::EndMap;

	emitter << YAML::EndMap;

	std::ofstream outFile(fileConfig);
	if(!outFile.is_open()) {
		UI_ERROR("[CfgApp::writeFileConfig] Fail to open the file");
		return;
	}
	outFile << emitter.c_str();
}

int CfgApp::getCameraFocusFromString(std::string str)
{
	for(const auto& pair : cameraFocusToString) {
		if(pair.second == str) {
			return (int) pair.first;
		}
	}
	return (int) TOMO_POSE;
}

int CfgApp::convertAccessLevel(std::string str)
{
	auto it = NameToLevelSignal.find(str);
	if(it != NameToLevelSignal.end()) {
		return it->second;
	}
	return ADMINISTRATOR;  // Default to Administrator if not found
}
