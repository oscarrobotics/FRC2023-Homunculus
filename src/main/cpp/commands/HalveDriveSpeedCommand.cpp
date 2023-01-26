#include "commands/HalveDriveSpeedCommand.h"

HalveDriveSpeedCommand::HalveDriveSpeedCommand(Drivetrain *subsystem)
    : m_drivetrain(subsystem) {}

void HalveDriveSpeedCommand::Initialize()
{
  m_drivetrain->SetMaxOutput(0.5);
}

void HalveDriveSpeedCommand::End(bool interrupted)
{
  m_drivetrain->SetMaxOutput(1);
}