#include "commands/DefaultArcadeDriveCommand.h"

DefaultArcadeDriveCommand::DefaultArcadeDriveCommand(Drivetrain *subsystem, std::function<double()> forward, std::function<double()> rotation)
    : m_drivetrain{subsystem}, m_forward{std::move(forward)}, m_rotation{std::move(rotation)}
{
  AddRequirements({subsystem});
}

void DefaultArcadeDriveCommand::Execute()
{
  m_drivetrain->ArcadeDrive(m_forward(), m_rotation());
}
