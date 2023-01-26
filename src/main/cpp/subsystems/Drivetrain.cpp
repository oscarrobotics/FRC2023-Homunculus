#include "subsystems/Drivetrain.h"

Drivetrain::Drivetrain() : m_leftMaster{kLeftMasterCANID},
                           m_leftDrone{kLeftDroneCANID},
                           m_rightMaster{kRightMasterCANID},
                           m_rightDrone{kRightDroneCANID}
{
  m_rightMotors.SetInverted(true);
}

void Drivetrain::Periodic()
{
}

void Drivetrain::SimulationPeriodic()
{
}

void Drivetrain::ArcadeDrive(double forward, double rotation)
{
  m_diffDrive.ArcadeDrive(forward, rotation);
}

void Drivetrain::SetMaxOutput(double maxOutput)
{
  m_diffDrive.SetMaxOutput(maxOutput);
}