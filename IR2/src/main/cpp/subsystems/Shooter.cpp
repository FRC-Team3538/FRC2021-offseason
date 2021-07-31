#include "subsystems/Shooter.hpp"

Shooter::Shooter()
{
}

void Shooter::SetFeeder(double speed)
{
    speed = std::min(speed, 1.0);
    speed = std::max(speed, -1.0);
    feeder._Set(speed);
}

void Shooter::SetTurretAngle(units::degrees_t targetAngle)
{
    targetAngle = std::min(targetAngle, maxTurretAngle);
    targetTurretAngle = std::max(targetAngle, minTurretAngle);
}

void Shooter::SetShooterVelocity(units::revolutions_per_minute_t targetRPM)
{
    targetShooterVelocity = std::min(targetRPM, maxFlywheelVelocity);
}

void Shooter::SetHood(double speed)
{
}

void Shooter::SetHoodAngle(units::degrees_t targetAngle)
{
    targetAngle = std::min(targetAngle, maxHoodAngle);
    targetHoodAngle = std::max(targetAngle, minHoodAngle);
}

units::degrees_t Shooter::GetHoodAngle()
{
    double ang = -hoodEncAbs.GetDistance() + offset;
    if (ang < 0 || ang > 120)
    {
        ang = fmod(ang, 144.0);
    }
    if (ang < 0)
    {
        ang += 144.0;
    }
    return (ang);
}

units::degrees_t Shooter::GetTurretAngle()
{
    double ang = -hoodEncAbs.GetDistance() + offset;
    if (ang < 0 || ang > 120)
    {
        ang = fmod(ang, 144.0);
    }
    if (ang < 0)
    {
        ang += 144.0;
    }
    return (ang);
}

units::revolutions_per_minute_t Shooter::GetShooterVelocity()
{
    return units::revolutions_per_minute_t{shooterA.GetSelectedSensorVelocity() * kScaleFactorFly * 600.0};
}

void Shooter::Periodic()
{
}