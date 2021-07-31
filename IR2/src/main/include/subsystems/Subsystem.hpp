#pragma once

class Subsystem
{
protected:
    virtual void UpdateTelemetry() = 0; 
    virtual void ConfigureMotors() = 0;
};