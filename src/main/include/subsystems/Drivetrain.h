#pragma once

#include <frc/Encoder.h>
#include <frc/drive/DifferentialDrive.h>
#include <frc/motorcontrol/MotorControllerGroup.h>
#include <frc/motorcontrol/PWMTalonFX.h>
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

  frc::PWMTalonFX m_leftMaster{kLeftMasterCANID};
  frc::PWMTalonFX m_leftDrone{kLeftDroneCANID};
  frc::PWMTalonFX m_rightMaster{kRightMasterCANID};
  frc::PWMTalonFX m_rightDrone{kRightDroneCANID};

  frc::MotorControllerGroup m_leftMotors{m_leftMaster, m_leftDrone};
  frc::MotorControllerGroup m_rightMotors{m_rightMaster, m_rightDrone};
};
