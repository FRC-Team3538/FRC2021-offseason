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
    void SetTurretAngle(units::degrees_t targetAngle);
    void SetShooterVelocity(units::revolutions_per_minute_t targetRPM);
    void SetHood(double speed);
    void SetHoodAngle(units::degrees_t targetAngle);

    // Getters
    units::degrees_t GetHoodAngle();
    units::degrees_t GetTurretAngle();
    units::revolutions_per_minute_t GetShooterVelocity();

    // Periodic
    void Periodic();

private:
    const std::array<units::revolutions_per_second_t, 5> velocitySetpoints{
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
    
    static constexpr units::revolutions_per_second_t maxFlywheelVelocity = 5500_rpm;
    static constexpr units::degree_t maxHoodAngle = 70_deg;
    static constexpr units::degree_t minHoodAngle = 15_deg;
    static constexpr units::degree_t maxTurretAngle = 50_deg;
    static constexpr units::degree_t minTurretAngle = -220_deg;
    static constexpr units::degree_t hoodZeroAngle = 0_deg;

    LazyTalonFX feeder{10};
    LazyTalonFX turret{11};
    LazyTalonFX shooterA{12};
    LazyTalonFX shooterB{13};

    frc::Servo hoodA{0};

    frc::DutyCycleEncoder hoodEncAbs;
    frc::DutyCycleEncoder turretEncAbs;

    units::degrees_t targetHoodAngle = 0.0_deg;
    units::degrees_t targetTurretAngle = 0.0_deg;
    units::revolutions_per_minute_t targetShooterVelocity = 0.0_rpm;
};