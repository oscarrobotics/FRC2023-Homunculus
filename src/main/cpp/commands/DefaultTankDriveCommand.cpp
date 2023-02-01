#include "commands/DefaultTankDriveCommand.h"

DefaultTankDriveCommand::DefaultTankDriveCommand(Drivetrain *subsystem, std::function<double()> left, std::function<double()> right)
    : m_drivetrain{subsystem}, m_left{std::move(left)}, m_right{std::move(right)}
{
  AddRequirements({subsystem});
}

void DefaultTankDriveCommand::Execute()
{
  m_drivetrain->TankDrive(m_left(), m_right());
}
