#pragma once

#include <frc/Encoder.h>
#include <frc/drive/DifferentialDrive.h>
#include <frc/motorcontrol/MotorControllerGroup.h>
#include <ctre/phoenix/motorcontrol/can/WPI_TalonFX.h>
#include <frc2/command/CommandPtr.h>
#include <frc2/command/SubsystemBase.h>

class Drivetrain : public frc2::SubsystemBase
{
  public:
  Drivetrain();

  void Periodic() override;
  void SimulationPeriodic() override;

  void ArcadeDrive(double forward, double rotation);
  void TankDrive(double left, double right);

  void SetMaxOutput(double maxOutput);

  private:
  int kLeftMasterCANID = 1;
  int kLeftDroneCANID = 2;
  int kRightMasterCANID = 3;
  int kRightDroneCANID = 4;

  frc::DifferentialDrive m_diffDrive{m_leftMotors, m_rightMotors};

  ctre::phoenix::motorcontrol::can::WPI_TalonFX m_leftMaster{kLeftMasterCANID};
  ctre::phoenix::motorcontrol::can::WPI_TalonFX m_leftDrone{kLeftDroneCANID};
  ctre::phoenix::motorcontrol::can::WPI_TalonFX m_rightMaster{kRightMasterCANID};
  ctre::phoenix::motorcontrol::can::WPI_TalonFX m_rightDrone{kRightDroneCANID};

  frc::MotorControllerGroup m_leftMotors{m_leftMaster, m_leftDrone};
  frc::MotorControllerGroup m_rightMotors{m_rightMaster, m_rightDrone};
};
