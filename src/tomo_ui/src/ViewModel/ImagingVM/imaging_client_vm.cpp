#include "imaging_client_vm.h"

ImagingClientVM::ImagingClientVM(QObject* parent, const std::string& serverName) : QObject(parent)
{
	bool status = startActionClient("imaging_client_ui");
}

ImagingClientVM::~ImagingClientVM()
{
}

bool ImagingClientVM::requestImagingAlgo(int imagingId, int actionId, int timeoutSec, std::vector<double> roiPose)
{
	ROS_INFO("Requesting imaging Algo: %s, Action ID: %d within %d secs", ImagingAlgoToName[static_cast<ImagingAlgoId>(imagingId)].c_str(),
			 actionId, timeoutSec);

	tomo_action::TomoImagingGoal goal;
	goal.pid	   = imagingId;
	goal.algo_name = ImagingAlgoToName[static_cast<ImagingAlgoId>(imagingId)];
	goal.action_id = actionId;

	// Send Goal
	_client->sendGoal(goal);

	// Get Result
	if(waitForSuccessResult(timeoutSec)) {
		// tomo_action::TomoImagingResult::ConstPtr result = _client->getResult();
		// std::vector<int> pimResult = result->PIM_counting_result;
		// ROS_INFO("Imaging Action Client: Algo RESULT: %s", result->PIM_status ? "true" : "false");
		// if(!result->PIM_status)
		// 	return false;
		// else {
		// 	for(auto i : pimResult) {
		// 		std::cout << i;
		// 	}
		// 	std::cout << std::endl;
		// 	return true;
		// }
	}
	else
		return false;
}
