#include "subsystems/Shooter.hpp"

Shooter::Shooter()
{
}

void Shooter::ConfigureMotors()
{

}

void Shooter::UpdateTelemetry()
{
    
}

/**
 * Sets % Output of Feeder Motor
 * 
 * @param speed Output from -1.0 to 1.0
 */
void Shooter::SetFeeder(double speed)
{
    speed = std::min(speed, 1.0);
    speed = std::max(speed, -1.0);
    feeder._Set(speed);
}

/**
 * Sets the target turret angle
 * 
 * @param targetAngle The target turret angle constrained by the min and max
 */
void Shooter::SetTurretAngle(units::degree_t targetAngle)
{
    targetAngle = std::min(targetAngle, maxTurretAngle);
    targetTurretAngle = std::max(targetAngle, minTurretAngle);
}

/**
 * Sets the target flywheel velocity
 * 
 * @param targetRPM The target flywheel velocity constrained by the min and max
 */
void Shooter::SetShooterVelocity(units::revolutions_per_minute_t targetRPM)
{
    targetShooterVelocity = std::min(targetRPM, maxFlywheelVelocity);
}

void Shooter::SetHood(double speed)
{
}

/**
 * Sets the target hood angle
 * 
 * @param targetAngle The target hood angle constrained by the min and max
 */
void Shooter::SetHoodAngle(units::degree_t targetAngle)
{
    targetAngle = std::min(targetAngle, maxHoodAngle);
    targetHoodAngle = std::max(targetAngle, minHoodAngle);
}

/**
 * Returns the current hood angle
 * 
 * @return units::degree_t The current hood angle
 */
units::degree_t Shooter::GetHoodAngle()
{
    double ang = -hoodEncAbs.GetDistance();// + offset;
    if (ang < 0 || ang > 120)
    {
        ang = fmod(ang, 144.0);
    }
    if (ang < 0)
    {
        ang += 144.0;
    }
    return units::degree_t(ang);
}

/**
 * Returns the current turret angle
 * 
 * @return units::degree_t The current turret angle
 */
units::degree_t Shooter::GetTurretAngle()
{
    double ang = -hoodEncAbs.GetDistance();// + offset;
    if (ang < 0 || ang > 120)
    {
        ang = fmod(ang, 144.0);
    }
    if (ang < 0)
    {
        ang += 144.0;
    }
    return units::degree_t(ang);
}

/**
 * Returns the current shooter velocity
 * 
 * @return units::revolutions_per_minute_t The current flywheel rpm
 */
units::revolutions_per_minute_t Shooter::GetShooterVelocity()
{
    return units::revolutions_per_minute_t{shooterA.GetSelectedSensorVelocity() * kScaleFactorFly * 600.0};
}

void Shooter::Periodic()
{
}