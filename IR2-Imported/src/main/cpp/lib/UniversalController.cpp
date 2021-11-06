/*----------------------------------------------------------------------------*/
/* Copyright (c) 2016-2018 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

#include "lib/UniversalController.hpp"

#include "hal/FRCUsageReporting.h"

using namespace frc;

/**
 * Construct an instance of a ps_4 controller.
 *
 * The controller index is the USB port on the Driver Station.
 *
 * @param port The port on the Driver Station that the controller is plugged
 *             into (0-5).
 */
UniversalController::UniversalController(int port) : GenericHID(port),
                                                     xb_{port},
                                                     ps_{port}//,
//                                                      stadia_{port}
{
  HAL_Report(HALUsageReporting::kResourceType_Joystick, port);
}

/**
 * Set the physical controller type
 *
 * @param type ControllerType to use {kXbox, kPS4, kStadia}
 */
void UniversalController::SetControllerType(ControllerType type)
{
  m_type = type;
}

/**
 * Get the X axis value of the controller.GenericHID::JoystickHand::kLeftHand
 *
 * @param hand Side of controller whose value should be returned.
 */
double UniversalController::GetLeftX() const
{
  switch (m_type)
  {
  case ControllerType::kXbox:
    return xb_.GetLeftX();
  case ControllerType::kPS4:
    return ps_.GetLeftX();
  case ControllerType::kStadia:
//     return stadia_.GetLeftX();
  default:
    return 0.0;
  }
}

/**
 * Get the Y axis value of the controller.
 *
 * @param hand Side of controller whose value should be returned.
 */
double UniversalController::GetLeftY() const
{
  switch (m_type)
  {
  case ControllerType::kXbox:
    return xb_.GetLeftY();
  case ControllerType::kPS4:
    return ps_.GetLeftY();
  case ControllerType::kStadia:
//     return stadia_.GetY(hand);
  default:
    return 0.0;
  }
}

/**
 * Get the trigger axis value of the controller.
 *
 * @param hand Side of controller whose value should be returned.
 */
double UniversalController::GetLeftTriggerAxis() const
{
  switch (m_type)
  {
  case ControllerType::kXbox:
    return xb_.GetLeftTriggerAxis();
  case ControllerType::kPS4:
    return ps_.GetL2Axis();
  case ControllerType::kStadia:
//     return stadia_.GetTriggerAxis(hand);
  default:
    return 0.0;
  }
}

/**
 * Read the value of the bumper button on the controller.
 *
 * @param hand Side of controller whose value should be returned.
 */
bool UniversalController::GetLeftBumper() const
{
  switch (m_type)
  {
  case ControllerType::kXbox:
    return xb_.GetLeftBumper();
  case ControllerType::kPS4:
    return ps_.GetL1Button();
  case ControllerType::kStadia:
//     return stadia_.GetBumper(hand);
  default:
    return false;
  }
}

/**
 * Whether the bumper was pressed since the last check.
 *
 * @param hand Side of controller whose value should be returned.
 * @return Whether the button was pressed since the last check.
 */
bool UniversalController::GetLeftBumperPressed()
{
  switch (m_type)
  {
  case ControllerType::kXbox:
    return xb_.GetLeftBumperPressed();
  case ControllerType::kPS4:
    return ps_.GetL1ButtonPressed();
  case ControllerType::kStadia:
//     return stadia_.GetBumperPressed(hand);
  default:
    return false;
  }
}

/**
 * Whether the bumper was released since the last check.
 *
 * @param hand Side of controller whose value should be returned.
 * @return Whether the button was released since the last check.
 */
bool UniversalController::GetLeftBumperReleased()
{
  switch (m_type)
  {
  case ControllerType::kXbox:
    return xb_.GetLeftBumperReleased();
  case ControllerType::kPS4:
    return ps_.GetL1ButtonReleased();
  case ControllerType::kStadia:
//     return stadia_.GetBumperReleased(hand);
  default:
    return false;
  }
}

/**
 * Read the value of the stick button on the controller.
 *
 * @param hand Side of controller whose value should be returned.
 * @return The state of the button.
 */
bool UniversalController::GetLeftStickButton() const
{
  switch (m_type)
  {
  case ControllerType::kXbox:
    return xb_.GetLeftStickButton();
  case ControllerType::kPS4:
    return ps_.GetL3Button();
  case ControllerType::kStadia:
//     return stadia_.GetStickButton(hand);
  default:
    return false;
  }
}

/**
 * Whether the stick button was pressed since the last check.
 *
 * @param hand Side of controller whose value should be returned.
 * @return Whether the button was pressed since the last check.
 */
bool UniversalController::GetLeftStickButtonPressed()
{
  switch (m_type)
  {
  case ControllerType::kXbox:
    return xb_.GetLeftStickButtonPressed();
  case ControllerType::kPS4:
    return ps_.GetL3ButtonPressed();
  case ControllerType::kStadia:
//     return stadia_.GetStickButtonPressed(hand);
  default:
    return false;
  }
}

/**
 * Whether the stick button was released since the last check.
 *
 * @param hand Side of controller whose value should be returned.
 * @return Whether the button was released since the last check.
 */
bool UniversalController::GetLeftStickButtonReleased()
{
  switch (m_type)
  {
  case ControllerType::kXbox:
    return xb_.GetLeftStickButtonReleased();
  case ControllerType::kPS4:
    return ps_.GetL3ButtonReleased();
  case ControllerType::kStadia:
//     return stadia_.GetStickButtonReleased(hand);
  default:
    return false;
  }
}

/**
 * Read the value of the Cross button on the controller.
 *
 * @return The state of the button.
 */
bool UniversalController::GetCrossButton() const
{
  switch (m_type)
  {
  case ControllerType::kXbox:
    return xb_.GetAButton();
  case ControllerType::kPS4:
    return ps_.GetCrossButton();
  case ControllerType::kStadia:
//     return stadia_.GetAButton();
  default:
    return false;
  }
}

/**
 * Whether the Cross button was pressed since the last check.
 *
 * @return Whether the button was pressed since the last check.
 */
bool UniversalController::GetCrossButtonPressed()
{
  switch (m_type)
  {
  case ControllerType::kXbox:
    return xb_.GetAButtonPressed();
  case ControllerType::kPS4:
    return ps_.GetCrossButtonPressed();
  case ControllerType::kStadia:
//     return stadia_.GetAButtonPressed();
  default:
    return false;
  }
}

/**
 * Whether the Cross button was released since the last check.
 *
 * @return Whether the button was released since the last check.
 */
bool UniversalController::GetCrossButtonReleased()
{
  switch (m_type)
  {
  case ControllerType::kXbox:
    return xb_.GetAButtonReleased();
  case ControllerType::kPS4:
    return ps_.GetCrossButtonReleased();
  case ControllerType::kStadia:
//     return stadia_.GetAButtonReleased();
  default:
    return false;
  }
}

/**
 * Read the value of the Circle button on the controller.
 *
 * @return The state of the button.
 */
bool UniversalController::GetCircleButton() const
{
  switch (m_type)
  {
  case ControllerType::kXbox:
    return xb_.GetBButton();
  case ControllerType::kPS4:
    return ps_.GetCircleButton();
  case ControllerType::kStadia:
//     return stadia_.GetBButton();
  default:
    return false;
  }
}

/**
 * Whether the Circle button was pressed since the last check.
 *
 * @return Whether the button was pressed since the last check.
 */
bool UniversalController::GetCircleButtonPressed()
{
  switch (m_type)
  {
  case ControllerType::kXbox:
    return xb_.GetBButtonPressed();
  case ControllerType::kPS4:
    return ps_.GetCircleButtonPressed();
  case ControllerType::kStadia:
//     return stadia_.GetBButtonPressed();
  default:
    return false;
  }
}

/**
 * Whether the Circle button was released since the last check.
 *
 * @return Whether the button was released since the last check.
 */
bool UniversalController::GetCircleButtonReleased()
{
  switch (m_type)
  {
  case ControllerType::kXbox:
    return xb_.GetBButtonReleased();
  case ControllerType::kPS4:
    return ps_.GetCircleButtonReleased();
  case ControllerType::kStadia:
//     return stadia_.GetBButtonReleased();
  default:
    return false;
  }
}

/**
 * Read the value of the Square button on the controller.
 *
 * @return The state of the button.
 */
bool UniversalController::GetSquareButton() const
{
  switch (m_type)
  {
  case ControllerType::kXbox:
    return xb_.GetXButton();
  case ControllerType::kPS4:
    return ps_.GetSquareButton();
  case ControllerType::kStadia:
//     return stadia_.GetXButton();
  default:
    return false;
  }
}

/**
 * Whether the Square button was pressed since the last check.
 *
 * @return Whether the button was pressed since the last check.
 */
bool UniversalController::GetSquareButtonPressed()
{
  switch (m_type)
  {
  case ControllerType::kXbox:
    return xb_.GetXButtonPressed();
  case ControllerType::kPS4:
    return ps_.GetSquareButtonPressed();
  case ControllerType::kStadia:
//     return stadia_.GetXButtonPressed();
  default:
    return false;
  }
}

/**
 * Whether the Square button was released since the last check.
 *
 * @return Whether the button was released since the last check.
 */
bool UniversalController::GetSquareButtonReleased()
{
  switch (m_type)
  {
  case ControllerType::kXbox:
    return xb_.GetXButtonReleased();
  case ControllerType::kPS4:
    return ps_.GetSquareButtonReleased();
  case ControllerType::kStadia:
//     return stadia_.GetXButtonReleased();
  default:
    return false;
  }
}

/**
 * Read the value of the Square button on the controller.
 *
 * @return The state of the button.
 */
bool UniversalController::GetTriangleButton() const
{
  switch (m_type)
  {
  case ControllerType::kXbox:
    return xb_.GetYButton();
  case ControllerType::kPS4:
    return ps_.GetTriangleButton();
  case ControllerType::kStadia:
//     return stadia_.GetYButton();
  default:
    return false;
  }
}

/**
 * Whether the Square button was pressed since the last check.
 *
 * @return Whether the button was pressed since the last check.
 */
bool UniversalController::GetTriangleButtonPressed()
{
  switch (m_type)
  {
  case ControllerType::kXbox:
    return xb_.GetYButtonPressed();
  case ControllerType::kPS4:
    return ps_.GetTriangleButtonPressed();
  case ControllerType::kStadia:
//     return stadia_.GetYButtonPressed();
  default:
    return false;
  }
}

/**
 * Whether the Square button was released since the last check.
 *
 * @return Whether the button was released since the last check.
 */
bool UniversalController::GetTriangleButtonReleased()
{
  switch (m_type)
  {
  case ControllerType::kXbox:
    return xb_.GetYButtonReleased();
  case ControllerType::kPS4:
    return ps_.GetTriangleButtonReleased();
  case ControllerType::kStadia:
//     return stadia_.GetYButtonReleased();
  default:
    return false;
  }
}

/**
 * Read the value of the Share button on the controller.
 *
 * @param hand Side of controller whose value should be returned.
 * @return The state of the button.
 */
bool UniversalController::GetShareButton() const
{
  switch (m_type)
  {
  case ControllerType::kXbox:
    return xb_.GetBackButton();
  case ControllerType::kPS4:
    return ps_.GetShareButton();
  case ControllerType::kStadia:
//     return stadia_.GetOptionsButton();
  default:
    return false;
  }
}

/**
 * Whether the Share button was pressed since the last check.
 *
 * @return Whether the button was pressed since the last check.
 */
bool UniversalController::GetShareButtonPressed()
{
  switch (m_type)
  {
  case ControllerType::kXbox:
    return xb_.GetBackButtonPressed();
  case ControllerType::kPS4:
    return ps_.GetShareButtonPressed();
  case ControllerType::kStadia:
//     return stadia_.GetOptionsButtonPressed();
  default:
    return false;
  }
}

/**
 * Whether the Share button was released since the last check.
 *
 * @return Whether the button was released since the last check.
 */
bool UniversalController::GetShareButtonReleased()
{
  switch (m_type)
  {
  case ControllerType::kXbox:
    return xb_.GetBackButtonReleased();
  case ControllerType::kPS4:
    return ps_.GetShareButtonReleased();
  case ControllerType::kStadia:
//     return stadia_.GetOptionsButtonReleased();
  default:
    return false;
  }
}

/**
 * Read the value of the Options button on the controller.
 *
 * @param hand Side of controller whose value should be returned.
 * @return The state of the button.
 */
bool UniversalController::GetOptionsButton() const
{
  switch (m_type)
  {
  case ControllerType::kXbox:
    return xb_.GetStartButton();
  case ControllerType::kPS4:
    return ps_.GetOptionsButton();
  case ControllerType::kStadia:
//     return stadia_.GetMenuButton();
  default:
    return false;
  }
}

/**
 * Whether the Options button was pressed since the last check.
 *
 * @return Whether the button was pressed since the last check.
 */
bool UniversalController::GetOptionsButtonPressed()
{
  switch (m_type)
  {
  case ControllerType::kXbox:
    return xb_.GetStartButtonPressed();
  case ControllerType::kPS4:
    return ps_.GetOptionsButtonPressed();
  case ControllerType::kStadia:
//     return stadia_.GetMenuButtonPressed();
  default:
    return false;
  }
}

/**
 * Whether the Options button was released since the last check.
 *
 * @return Whether the button was released since the last check.
 */
bool UniversalController::GetOptionsButtonReleased()
{
  switch (m_type)
  {
  case ControllerType::kXbox:
    return xb_.GetStartButtonReleased();
  case ControllerType::kPS4:
    return ps_.GetOptionsButtonReleased();
  case ControllerType::kStadia:
//     return stadia_.GetMenuButtonReleased();
  default:
    return false;
  }
}

/**
 * Read the value of the ps_ button on the controller.
 *
 * @param hand Side of controller whose value should be returned.
 * @return The state of the button.
 */
bool UniversalController::GetPSButton() const
{
  switch (m_type)
  {
  case ControllerType::kXbox:
    return false;
  case ControllerType::kPS4:
    return ps_.GetPSButton();
  case ControllerType::kStadia:
//     return stadia_.GetStadiaButton();
  default:
    return false;
  }
}

/**
 * Whether the ps_ button was pressed since the last check.
 *
 * @return Whether the button was pressed since the last check.
 */
bool UniversalController::GetPSButtonPressed()
{
  switch (m_type)
  {
  case ControllerType::kXbox:
    return false;
  case ControllerType::kPS4:
    return ps_.GetPSButtonPressed();
  case ControllerType::kStadia:
//     return stadia_.GetStadiaButtonPressed();
  default:
    return false;
  }
}

/**
 * Whether the ps_ button was released since the last check.
 *
 * @return Whether the button was released since the last check.
 */
bool UniversalController::GetPSButtonReleased()
{
  switch (m_type)
  {
  case ControllerType::kXbox:
    return false;
  case ControllerType::kPS4:
    return ps_.GetPSButtonReleased();
  case ControllerType::kStadia:
//     return stadia_.GetStadiaButtonReleased();
  default:
    return false;
  }
}

/**
 * Read the value of the TouchPad button on the controller.
 *
 * @param hand Side of controller whose value should be returned.
 * @return The state of the button.
 */
bool UniversalController::GetTouchPadButton() const
{
  switch (m_type)
  {
  case ControllerType::kXbox:
    return false;
  case ControllerType::kPS4:
    return ps_.GetTouchpad();
  case ControllerType::kStadia:
//     return stadia_.GetCaptureButton();
  default:
    return false;
  }
}

/**
 * Whether the TouchPad button was pressed since the last check.
 *
 * @return Whether the button was pressed since the last check.
 */
bool UniversalController::GetTouchPadButtonPressed()
{
  switch (m_type)
  {
  case ControllerType::kXbox:
    return false;
  case ControllerType::kPS4:
    return ps_.GetTouchpadPressed();
  case ControllerType::kStadia:
//     return stadia_.GetCaptureButtonPressed();
  default:
    return false;
  }
}

/**
 * Whether the ps_ button was released since the last check.
 *
 * @return Whether the button was released since the last check.
 */
bool UniversalController::GetTouchPadButtonReleased()
{
  switch (m_type)
  {
  case ControllerType::kXbox:
    return false;
  case ControllerType::kPS4:
    return ps_.GetTouchpadReleased();
  case ControllerType::kStadia:
//     return stadia_.GetCaptureButtonReleased();
  default:
    return false;
  }
}

// TODO: Add xb_ox Getter functions

/**
 * Read the value of the TouchPad button on the controller.
 *
 * @param hand Side of controller whose value should be returned.
 * @return The state of the button.
 */
bool UniversalController::GetUpButton() const
{
  return (GetPOV() == 315 || GetPOV() == 0 || GetPOV() == 45);
}

/**
 * Read the value of the TouchPad button on the controller.
 *
 * @param hand Side of controller whose value should be returned.
 * @return The state of the button.
 */
bool UniversalController::GetRightButton() const
{
  return (GetPOV() == 45 || GetPOV() == 90 || GetPOV() == 135);
}

/**
 * Read the value of the TouchPad button on the controller.
 *
 * @param hand Side of controller whose value should be returned.
 * @return The state of the button.
 */
bool UniversalController::GetDownButton() const
{
  return (GetPOV() == 135 || GetPOV() == 180 || GetPOV() == 225);
}

/**
 * Read the value of the TouchPad button on the controller.
 *
 * @param hand Side of controller whose value should be returned.
 * @return The state of the button.
 */
bool UniversalController::GetLeftButton() const
{
  return (GetPOV() == 225 || GetPOV() == 270 || GetPOV() == 315);
}

/**
 * Generate a SmartDash object interface
 *
 * @param hand Side of controller whose value should be returned.
 * @return The state of the button.
 */
void UniversalController::InitSendable(wpi::SendableBuilder &builder)
{
  builder.SetSmartDashboardType("UniversalController");
  builder.SetActuator(true);

  // Axis
  builder.AddDoubleProperty(
      "axis/LX", [this] { return GetLeftX(); }, nullptr);
  builder.AddDoubleProperty(
      "axis/LY", [this] { return GetLeftY(); }, nullptr);
  builder.AddDoubleProperty(
      "axis/LT", [this] { return GetLeftTriggerAxis(); }, nullptr);
  builder.AddDoubleProperty(
      "axis/RX", [this] { return GetRightX(); }, nullptr);
  builder.AddDoubleProperty(
      "axis/RY", [this] { return GetRightY(); }, nullptr);
  builder.AddDoubleProperty(
      "axis/RT", [this] { return GetRightTriggerAxis(); }, nullptr);
  builder.AddDoubleProperty(
      "axis/POV", [this] { return GetPOV(); }, nullptr);

  // Button
  builder.AddBooleanProperty(
      "btn/Cross", [this] { return GetCrossButton(); }, nullptr);
  builder.AddBooleanProperty(
      "btn/Circle", [this] { return GetCircleButton(); }, nullptr);
  builder.AddBooleanProperty(
      "btn/Square", [this] { return GetSquareButton(); }, nullptr);
  builder.AddBooleanProperty(
      "btn/Triangle", [this] { return GetTriangleButton(); }, nullptr);

  builder.AddBooleanProperty(
      "btn/BumperL", [this] { return GetLeftBumper(); }, nullptr);
  builder.AddBooleanProperty(
      "btn/BumperR", [this] { return GetRightBumper(); }, nullptr);

  builder.AddBooleanProperty(
      "btn/StickL", [this] { return GetLeftStickButton(); }, nullptr);
  builder.AddBooleanProperty(
      "btn/StickR", [this] { return GetRightStickButton(); }, nullptr);

  builder.AddBooleanProperty(
      "btn/Share", [this] { return GetShareButton(); }, nullptr);
  builder.AddBooleanProperty(
      "btn/Options", [this] { return GetOptionsButton(); }, nullptr);
  builder.AddBooleanProperty(
      "btn/PS", [this] { return GetPSButton(); }, nullptr);
  builder.AddBooleanProperty(
      "btn/Touchpad", [this] { return GetTouchPadButton(); }, nullptr);
}