#include "subsystems/RJVisionPipeline.hpp"

using namespace nt;

namespace vision
{

    RJVisionPipeline::RJVisionPipeline()
    {
    }

    void RJVisionPipeline::Init()
    {
        table = nt::NetworkTableInstance::GetDefault().GetTable("limelight");
        table->PutNumber("ledMode", 1.0);
        table->PutNumber("pipeline", 0.0);
        table->PutNumber("camMode", 0.0);
    }

    void RJVisionPipeline::Periodic()
    {
        dx = -(table->GetNumber("tx", 0.0));
        dy = table->GetNumber("ty", 0.0);
        tv = table->GetNumber("tv", 0.0);
    }

    RJVisionPipeline::visionData RJVisionPipeline::Run()
    {
        if (table->GetNumber("ledMode", 0.0) != 3.0)
        {
            table->PutNumber("ledMode", 3.0);
            lightOn.Reset();
            lightOn.Start();
        }

        RJVisionPipeline::visionData telemetry;

        if (((pipeSwitch.Get() < 1.0) && pipeSwitchOS))
        {
            telemetry.angle = 420.0;
            telemetry.distance = -1.0;
            telemetry.filled = false;
        }
        else if ((pipeSwitch.Get() > 1.0 || !pipeSwitchOS) && (lightOn.Get() > 0.5))
        {
            if (tv == 1.0)
            {
                telemetry.angle = dx;
                telemetry.distance = DistEstimation();
                telemetry.filled = true;
            }
            else
            {
                telemetry.angle = 420.0;
                telemetry.distance = -1.0;
                telemetry.filled = false;
            }
        }
        return telemetry;
    }

    void RJVisionPipeline::UpdateSmartDash()
    {
        frc::SmartDashboard::PutNumber("dx", dx);
        frc::SmartDashboard::PutNumber("Vision Dist", estDist);
    }

    void RJVisionPipeline::Reset()
    {
        table->PutNumber("ledMode", 1.0);
        pipeSwitchOS = false;
        pipeSwitchCt = 0;
    }

    double RJVisionPipeline::DistEstimation()
    {
        double dist = dh / (tan((dy + cameraAngle) * (3.1415 / 180.0)));
        estDist = dist;
        return estDist;
    }
} // namespace vision
