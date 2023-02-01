// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#include "RobotContainer.h"

#include <frc/shuffleboard/Shuffleboard.h>
#include <frc2/command/button/JoystickButton.h>

#include "commands/DefaultArcadeDriveCommand.h"
#include "commands/UncapDriveSpeedCommand.h"

RobotContainer::RobotContainer()
{
  // m_autonomousChooser.SetDefaultOption("Some auto", &m_someAutoCommand);
  // m_autonomousChooser.AddOption("Some other auto", &m_someOtherAutoCommand);

  frc::Shuffleboard::GetTab("Autonomous").Add(m_autonomousChooser);

  ConfigureBindings();

  m_drivetrain.SetDefaultCommand(DefaultArcadeDriveCommand(
      &m_drivetrain, [this] { return -m_driverController.GetLeftY(); }, [this] { return -m_driverController.GetRightX(); }));
}

void RobotContainer::ConfigureBindings()
{
  frc2::JoystickButton(&m_driverController, frc::XboxController::Button::kRightBumper)
      .WhileTrue(UncapDriveSpeedCommand(&m_drivetrain).ToPtr());
}

frc2::Command *RobotContainer::GetAutonomousCommand()
{
  return m_autonomousChooser.GetSelected();
}
