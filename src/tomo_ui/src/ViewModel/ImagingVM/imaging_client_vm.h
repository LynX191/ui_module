#ifndef IMAGING_CLIENT_VM_H
#define IMAGING_CLIENT_VM_H
#pragma once

#include <QObject>
#include <actionlib/client/simple_action_client.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <tomo_action/TomoImagingAction.h>
#include <tomo_action/TomoImagingGoal.h>
#include <tomo_action/TomoImagingResult.h>
#include <tomo_comm/tomo_action_client.h>

using tImagingClient = actionlib::SimpleActionClient<tomo_action::TomoImagingAction>;
static std::vector<double> defaultPose(7, 0);
enum class ImagingAlgoId
{
	DEFAULT,					  //
	CARTON_VISION = 28,			  //
	SHIPPER_TRAY,				  //
	ALUMINUM_BOX,				  //
	SMALL_BOTTLE_CLASSIFICATION,  //
	PIM_CARTON_COUNTING,		  //
	BASE_SHIPPER_TRAY,			  //
};

static std::map<ImagingAlgoId, std::string> ImagingAlgoToName = {
	{ImagingAlgoId::DEFAULT, "Default"},										//
	{ImagingAlgoId::CARTON_VISION, "CartonVision"},								//
	{ImagingAlgoId::SHIPPER_TRAY, "ShipperTray"},								//
	{ImagingAlgoId::ALUMINUM_BOX, "AluminumBox"},								//
	{ImagingAlgoId::SMALL_BOTTLE_CLASSIFICATION, "SmallBottleClassification"},	//
	{ImagingAlgoId::PIM_CARTON_COUNTING, "PimCartonCounting"},					//
	{ImagingAlgoId::BASE_SHIPPER_TRAY, "BaseShipperTray"},						//
};

class ImagingClientVM : public QObject, public TomoActionClient<tImagingClient>
{
	Q_OBJECT

public:
	explicit ImagingClientVM(QObject* parent = nullptr, const std::string& serverName = "imaging_server");
	~ImagingClientVM();

public Q_SLOTS:
	bool requestImagingAlgo(int imagingId, int actionId, int timeoutSec = 10, std::vector<double> roiPose = defaultPose);

private:
};

#endif	// IMAGING_CLIENT_VM_H