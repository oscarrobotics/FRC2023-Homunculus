#include "commands/UncapDriveSpeedCommand.h"

UncapDriveSpeedCommand::UncapDriveSpeedCommand(Drivetrain *subsystem)
    : m_drivetrain(subsystem) {}

void UncapDriveSpeedCommand::Initialize()
{
  m_drivetrain->SetMaxOutput(1);
}

void UncapDriveSpeedCommand::End(bool interrupted)
{
  m_drivetrain->SetMaxOutput(0.5);
}