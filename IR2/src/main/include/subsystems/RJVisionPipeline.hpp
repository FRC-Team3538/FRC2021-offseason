#pragma once

#include <iostream>
#include "frc/smartdashboard/SmartDashboard.h"
#include "networktables/NetworkTable.h"
#include "networktables/NetworkTableInstance.h"
#include "networktables/NetworkTableEntry.h"
#include <cmath>
#include <frc/Timer.h>

namespace vision
{

/*
* RJVisionPipeline class.
*/

class RJVisionPipeline
{
private:
	const double cameraAngle = 32;
	const double dh = 63.0; //distance between camera lens and quarter-way up the goal

    double estDist = 0.0;

	std::shared_ptr<NetworkTable> table;
	double dy, dx, tv;
	frc::Timer lightOn;

    bool pipeSwitchOS = false;
	int pipeSwitchCt = 0;
	frc::Timer pipeSwitch;

public:
	struct visionData
	{
		double distance, angle;
		bool filled = false;
	};

    // Init Stuff
	RJVisionPipeline();
	void Init();

    // Periodic
	void Periodic();

    // Setter
	RJVisionPipeline::visionData Run();
	double DistEstimation();
	void Reset();

private:
    void UpdateSmartDash();
};
} // namespace vision
