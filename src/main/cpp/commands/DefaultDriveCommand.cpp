// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#include "commands/DefaultDriveCommand.h"

DefaultDriveCommand::DefaultDriveCommand(Drivetrain *subsystem, std::function<double()> forward, std::function<double()> rotation)
    : m_drivetrain{subsystem}, m_forward{std::move(forward)}, m_rotation{std::move(rotation)}
{
  AddRequirements({subsystem});
}

void DefaultDriveCommand::Execute()
{
  m_drivetrain->ArcadeDrive(m_forward(), m_rotation());
}
