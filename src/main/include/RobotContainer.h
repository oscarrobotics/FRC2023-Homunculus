// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#pragma once

#include <frc/smartdashboard/SendableChooser.h>
#include <frc2/command/button/CommandXboxController.h>

#include "subsystems/Drivetrain.h"

class RobotContainer
{
  public:
  RobotContainer();

  frc2::Command *GetAutonomousCommand();

  private:
  frc2::CommandXboxController m_driverController{0}; // TODO: un hardcode this

  frc::SendableChooser<frc2::Command *> m_autonomousChooser;

  Drivetrain m_drivetrain;

  void ConfigureBindings();
};
