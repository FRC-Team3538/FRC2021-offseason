#pragma once

//                                         ___    ,'""""'.
//                                     ,"""   """"'      `.
//                                    ,'        _.         `._
//                                   ,'       ,'              `"""'.
//                                  ,'    .-""`.    ,-'            `.
//                                 ,'    (        ,'                :
//                               ,'     ,'           __,            `.
//                         ,""""'     .' ;-.    ,  ,'  \             `"""".
//                       ,'           `-(   `._(_,'     )_                `.
//                      ,'         ,---. \ @ ;   \ @ _,'                   `.
//                 ,-""'         ,'      ,--'-    `;'                       `.
//                ,'            ,'      (      `. ,'                          `.
//                ;            ,'        \    _,','                            `.
//               ,'            ;          `--'  ,'                              `.
//              ,'             ;          __    (                    ,           `.
//              ;              `____...  `78b   `.                  ,'           ,'
//              ;    ...----'''' )  _.-  .d8P    `.                ,'    ,'    ,'
// _....----''' '.        _..--"_.-:.-' .'        `.             ,''.   ,' `--'
//               `"WIND"" _.-'' .-'`-.:..___...--' `-._      ,-"'   `-'
//         _.--'       _.-'    .'   .' .'               `"""""
//   __.-''        _.-'     .-'   .'  /
//  '          _.-' .-'  .-'        .'
//         _.-'  .-'  .-' .'  .'   /
//     _.-'      .-'   .-'  .'   .'
// _.-'       .-'    .'   .'    /  
//        _.-'    .-'   .'    .'
//     .-'            .'

// Units
#include <units/angular_velocity.h>
#include <units/angular_acceleration.h>
#include <units/velocity.h>
#include <units/current.h>
#include <units/acceleration.h>
#include <units/temperature.h>
#include <units/time.h>

// Utilities
#include "lib/LazyTalonFX.hpp"
#include "subsystems/Subsystem.hpp"
#include <frc/Servo.h>
#include <frc/DutyCycleEncoder.h>
#include <cmath>
#include <vector>
#include <frc/Solenoid.h>

class Shooter : public Subsystem
{
public:
    // Constructor
    Shooter();

    // Init Stuff
    void ConfigureMotors();

    // Telemetry
    void UpdateTelemetry();

    // Setters
    void SetFeeder(double speed);
    void SetTurretAngle(units::degree_t targetAngle);
    void SetShooterVelocity(units::revolutions_per_minute_t targetRPM);
    void AutoSetVelocity(units::inch_t distance);
    void SetHood(double speed);
    void SetHoodAngle(units::degree_t targetAngle);
    void SetTurret(double speed);

    // Getters
    units::degree_t GetHoodAngle();
    units::degree_t GetTurretAngle();
    units::revolutions_per_minute_t GetShooterVelocity();

    // Periodic
    void Periodic();

private:
    const std::array<units::revolutions_per_minute_t, 5> velocitySetpoints{
        0_rpm,
        2450_rpm,
        2450_rpm,
        3650_rpm,
        3250_rpm};

    const std::array<units::degree_t, 5> hoodSetpoints{
        0_deg,
        32.5_deg,
        38_deg,
        49.5_deg,
        47.5_deg};

    static constexpr double kScaleFactorFly = (1.0 / 2048);
    
    static constexpr units::revolutions_per_minute_t maxFlywheelVelocity = 5500_rpm;
    static constexpr units::degree_t maxHoodAngle = 70_deg;
    static constexpr units::degree_t minHoodAngle = 15_deg;
    static constexpr units::degree_t maxTurretAngle = 50_deg;
    static constexpr units::degree_t minTurretAngle = 140_deg;
    static constexpr units::degree_t hoodZeroAngle = 0_deg;
    
    double kScaleFactorTurret = 332.0 / ((555.0 / 11.0) * 2048.0); // Degrees / (Ratio * Ticks per Rev)

    LazyTalonFX feeder{10};
    LazyTalonFX turret{11};
    LazyTalonFX shooterA{12};
    LazyTalonFX shooterB{13};

    frc::Servo hoodA{0};

    frc::Solenoid feederSol{2};

    frc::DutyCycleEncoder hoodEncAbs{0};
    frc::DutyCycleEncoder turretEncAbs{1};

    units::degree_t targetHoodAngle = 0.0_deg;
    units::degree_t targetTurretAngle = 90.0_deg;
    units::revolutions_per_minute_t targetShooterVelocity = 0.0_rpm;

    double kP = 0.175;
    double kD = 0.001;
    units::degree_t prevErr = 0.0_deg;

    // Shooter Interpolation 
    const std::array<double, 4> interpolationVals{0, 0, 0, 0};

    double prevDist = -1.0;
    double prevRPM = -1.0;

    bool feed = false;
};