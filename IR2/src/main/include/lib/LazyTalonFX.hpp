#pragma once

#include <ctre/Phoenix.h>
#include <units/voltage.h>

class LazyTalonFX : public WPI_TalonFX
{
protected:
    double mLastSet = 420.0;
    units::voltage::volt_t mLastV = units::volt_t{420.0};
    const char *device = "Talon FX";

public:
    LazyTalonFX() = delete;
    LazyTalonFX(int ID) : BaseMotorController(ID, device), BaseTalon(ID, device), TalonFX(ID), WPI_BaseMotorController(ID, device), WPI_TalonFX(ID)
    {
        ConfigFactoryDefault();
    }

    /**
     * Sets Talon's control mode and associated value
     * 
     * @param mode is the control mode of the Talon
     * @param value is the input value based on the control mode
     */
    void _Set(double value)
    {
        if (value != mLastSet)
        {
            mLastSet = value;
            Set(value);
        }
    }

    void _SetVoltage(units::voltage::volt_t voltage)
    {
        if (voltage != mLastV)
        {
            mLastV = voltage;
            SetVoltage(voltage);
        }
    }
};