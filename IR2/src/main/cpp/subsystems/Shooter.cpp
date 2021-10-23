#include "subsystems/Shooter.hpp"

Shooter::Shooter()
{
}

void Shooter::ConfigureMotors()
{
    turret.ConfigFactoryDefault();
    turret.SetSelectedSensorPosition(90 / kScaleFactorTurret);
    turret.SetInverted(true);
    turret.SetNeutralMode(ctre::phoenix::motorcontrol::NeutralMode::Brake);
    turret.ConfigPeakOutputForward(0.2);
    turret.ConfigPeakOutputReverse(-0.2);

    shooterB.Follow(shooterA);

    shooterA.ConfigFactoryDefault();
    shooterA.SetInverted(true);

    shooterB.ConfigFactoryDefault();
    shooterB.SetInverted(false);

    shooterA.Config_kF(0, 0.056494409);
    shooterA.Config_kP(0, 0.225);
    shooterA.Config_kI(0, 0.0001);
    shooterA.Config_kD(0, 6.000);

    shooterA.Config_IntegralZone(0, 200.0);

    turret.SetNeutralMode(ctre::phoenix::motorcontrol::Brake);

    feeder.SetStatusFramePeriod(StatusFrameEnhanced::Status_1_General, 250, 50);
    feeder.SetStatusFramePeriod(StatusFrameEnhanced::Status_2_Feedback0, 250, 50);
    feeder.SetStatusFramePeriod(StatusFrameEnhanced::Status_4_AinTempVbat, 250, 50);
    feeder.SetStatusFramePeriod(StatusFrameEnhanced::Status_6_Misc, 250, 50);
    feeder.SetStatusFramePeriod(StatusFrameEnhanced::Status_7_CommStatus, 250, 50);
    feeder.SetStatusFramePeriod(StatusFrameEnhanced::Status_9_MotProfBuffer, 250, 50);
    feeder.SetStatusFramePeriod(StatusFrameEnhanced::Status_10_Targets, 250, 50);
    feeder.SetStatusFramePeriod(StatusFrameEnhanced::Status_12_Feedback1, 250, 50);
    feeder.SetStatusFramePeriod(StatusFrameEnhanced::Status_13_Base_PIDF0, 250, 50);
    feeder.SetStatusFramePeriod(StatusFrameEnhanced::Status_14_Turn_PIDF1, 250, 50);
    feeder.SetStatusFramePeriod(StatusFrameEnhanced::Status_15_FirmareApiStatus, 250, 50);
    feeder.SetStatusFramePeriod(StatusFrameEnhanced::Status_17_Targets1, 250, 50);
    feeder.SetStatusFramePeriod(StatusFrameEnhanced::Status_3_Quadrature, 250, 50);
    feeder.SetStatusFramePeriod(StatusFrameEnhanced::Status_8_PulseWidth, 250, 50);
    feeder.SetStatusFramePeriod(StatusFrameEnhanced::Status_11_UartGadgeteer, 250, 50);
    feeder.SetStatusFramePeriod(StatusFrameEnhanced::Status_Brushless_Current, 250, 50);

    shooterB.SetStatusFramePeriod(StatusFrameEnhanced::Status_1_General, 250, 50);
    shooterB.SetStatusFramePeriod(StatusFrameEnhanced::Status_2_Feedback0, 250, 50);
    shooterB.SetStatusFramePeriod(StatusFrameEnhanced::Status_4_AinTempVbat, 250, 50);
    shooterB.SetStatusFramePeriod(StatusFrameEnhanced::Status_6_Misc, 250, 50);
    shooterB.SetStatusFramePeriod(StatusFrameEnhanced::Status_7_CommStatus, 250, 50);
    shooterB.SetStatusFramePeriod(StatusFrameEnhanced::Status_9_MotProfBuffer, 250, 50);
    shooterB.SetStatusFramePeriod(StatusFrameEnhanced::Status_10_Targets, 250, 50);
    shooterB.SetStatusFramePeriod(StatusFrameEnhanced::Status_12_Feedback1, 250, 50);
    shooterB.SetStatusFramePeriod(StatusFrameEnhanced::Status_13_Base_PIDF0, 250, 50);
    shooterB.SetStatusFramePeriod(StatusFrameEnhanced::Status_14_Turn_PIDF1, 250, 50);
    shooterB.SetStatusFramePeriod(StatusFrameEnhanced::Status_15_FirmareApiStatus, 250, 50);
    shooterB.SetStatusFramePeriod(StatusFrameEnhanced::Status_17_Targets1, 250, 50);
    shooterB.SetStatusFramePeriod(StatusFrameEnhanced::Status_3_Quadrature, 250, 50);
    shooterB.SetStatusFramePeriod(StatusFrameEnhanced::Status_8_PulseWidth, 250, 50);
    shooterB.SetStatusFramePeriod(StatusFrameEnhanced::Status_11_UartGadgeteer, 250, 50);
    shooterB.SetStatusFramePeriod(StatusFrameEnhanced::Status_Brushless_Current, 250, 50);
}

void Shooter::UpdateTelemetry()
{
    frc::SmartDashboard::PutNumber("Turret Angle", GetTurretAngle().value());
    frc::SmartDashboard::PutNumber("Shooter Velocity", GetShooterVelocity().value());
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
    if (std::abs(speed) > 0.05)
        feed = true;
    else
        feed = false;
}

/**
 * Sets the target turret angle
 * 
 * @param targetAngle The target turret angle constrained by the min and max
 */
void Shooter::SetTurretAngle(units::degree_t targetAngle)
{
    // targetAngle = std::min(targetAngle, maxTurretAngle);
    // targetTurretAngle = std::max(targetAngle, minTurretAngle);
    // targetTurretAngle = 0.0_deg;
    targetTurretAngle = targetAngle;
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

void Shooter::AutoSetVelocity(units::inch_t distance)
{
    if (prevDist == distance.value())
    {
        SetShooterVelocity(units::revolutions_per_minute_t{prevRPM});
        return;
    }

    prevDist = distance.value();

    double rpm = 0.0;
    for (int i = 0; i < interpolationVals.size(); ++i)
    {
        rpm += interpolationVals[i] * std::pow(distance.value(), i);
    }

    SetShooterVelocity(units::revolutions_per_minute_t{rpm});
    prevRPM = rpm;

    if (std::abs(GetShooterVelocity().value() - targetShooterVelocity.value()) < 100.0)
    {
        SetFeeder(1.0);
    }
    else
    {
        SetFeeder(0.0);
    }
}

void Shooter::SetHood(double position)
{
    hoodA.Set(position);
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
    double ang = -hoodEncAbs.GetDistance(); // + offset;
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

void Shooter::SetTurret(double speed)
{
    turret._Set(speed);
    targetTurretAngle = GetTurretAngle();
}

/**
 * Returns the current turret angle
 * 
 * @return units::degree_t The current turret angle
 */
units::degree_t Shooter::GetTurretAngle()
{
    // double ang = -hoodEncAbs.GetDistance();// + offset;
    // if (ang < 0 || ang > 120)
    // {
    //     ang = fmod(ang, 144.0);
    // }
    // if (ang < 0)
    // {
    //     ang += 144.0;
    // }
    // return units::degree_t(ang);

    double ang = turret.GetSelectedSensorPosition() * kScaleFactorTurret;
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
    units::degree_t err = targetTurretAngle - GetTurretAngle();
    double dVal = (err.value() - prevErr.value()) / 0.02;
    //units::volt_t command = units::volt_t{(err.value() * kP) + (dVal * kD)};
    //turret._SetVoltage(command);
    prevErr = err;

    if (!feed)
        feederSol.Set(true);
    else
        feederSol.Set(false);

    if(targetShooterVelocity.value() == 0.0)
        shooterA.Set(0.0);
    else
        shooterA.Set(ControlMode::Velocity, ((targetShooterVelocity.value() / kScaleFactorFly) / 600.0));
}