/*----------------------------------------------------------------------------*/
/* Copyright (c) 2016-2018 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

#pragma once

#include <frc/GenericHID.h>
#include <frc/XboxController.h>
#include <frc/PS4Controller.h>
// #include "lib/StadiaController.hpp"

#include <wpi/sendable/Sendable.h>
#include <wpi/sendable/SendableBuilder.h>
#include <wpi/sendable/SendableHelper.h>

namespace frc
{

  /**
 * Handle input from various types of controllers and provide the same 
 * API as a PS4 Controller, since that's what we use. 
 * 
 * TODO: Support other controller APIs as well?
 */
  class UniversalController : public GenericHID,
                              public wpi::Sendable,
                              public wpi::SendableHelper<UniversalController>
  {
  public:
    explicit UniversalController(int port);
    virtual ~UniversalController() = default;

    UniversalController(const UniversalController &) = delete;
    UniversalController &operator=(const UniversalController &) = delete;

    enum class ControllerType
    {
      kXbox,
      kPS4,
      kStadia,
    };



    void SetControllerType(ControllerType type);

    double GetLeftX() const;
    double GetLeftY() const;
    double GetLeftTriggerAxis() const;

    double GetRightX() const;
    double GetRightY() const;
    double GetRightTriggerAxis() const;

    bool GetLeftBumper() const;
    bool GetLeftBumperPressed();
    bool GetLeftBumperReleased();

    bool GetRightBumper() const;
    bool GetRightBumperPressed();
    bool GetRightBumperReleased();

    bool GetLeftStickButton() const;
    bool GetLeftStickButtonPressed();
    bool GetLeftStickButtonReleased();

    bool GetRightStickButton() const;
    bool GetRightStickButtonPressed();
    bool GetRightStickButtonReleased();


    bool GetCrossButton() const;
    bool GetCrossButtonPressed();
    bool GetCrossButtonReleased();

    bool GetCircleButton() const;
    bool GetCircleButtonPressed();
    bool GetCircleButtonReleased();

    bool GetSquareButton() const;
    bool GetSquareButtonPressed();
    bool GetSquareButtonReleased();

    bool GetTriangleButton() const;
    bool GetTriangleButtonPressed();
    bool GetTriangleButtonReleased();

    bool GetShareButton() const;
    bool GetShareButtonPressed();
    bool GetShareButtonReleased();

    bool GetOptionsButton() const;
    bool GetOptionsButtonPressed();
    bool GetOptionsButtonReleased();

    bool GetPSButton() const;
    bool GetPSButtonPressed();
    bool GetPSButtonReleased();

    bool GetTouchPadButton() const;
    bool GetTouchPadButtonPressed();
    bool GetTouchPadButtonReleased();

    bool GetUpButton() const;
    bool GetRightButton() const;
    bool GetDownButton() const;
    bool GetLeftButton() const;

    // SmartDash Support
    void InitSendable(wpi::SendableBuilder &builder) override;

  private:
    ControllerType m_type = ControllerType::kPS4;

    XboxController xb_;
    PS4Controller ps_;
//     StadiaController stadia_;
  };

} // namespace frc