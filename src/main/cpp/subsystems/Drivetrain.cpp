#include "subsystems/Drivetrain.h"

#include <iostream>

Drivetrain::Drivetrain() : m_leftMaster{kLeftMasterCANID},
                           m_leftDrone{kLeftDroneCANID},
                           m_rightMaster{kRightMasterCANID},
                           m_rightDrone{kRightDroneCANID}
{
  m_rightMotors.SetInverted(true);
  this->SetMaxOutput(0.5);
  m_rightMaster.SetNeutralMode(ctre::phoenix::motorcontrol::NeutralMode::Brake);
  m_rightDrone.SetNeutralMode(ctre::phoenix::motorcontrol::NeutralMode::Brake);
  m_leftMaster.SetNeutralMode(ctre::phoenix::motorcontrol::NeutralMode::Brake);
  m_leftDrone.SetNeutralMode(ctre::phoenix::motorcontrol::NeutralMode::Brake);
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

void Drivetrain::TankDrive(double left, double right)
{
  m_diffDrive.TankDrive(left, right);
}

void Drivetrain::SetMaxOutput(double maxOutput)
{
  m_diffDrive.SetMaxOutput(maxOutput);
}