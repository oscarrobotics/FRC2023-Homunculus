#pragma once

#include <frc2/command/CommandBase.h>
#include <frc2/command/CommandHelper.h>

#include "subsystems/Drivetrain.h"

class DefaultArcadeDriveCommand : public frc2::CommandHelper<frc2::CommandBase, DefaultArcadeDriveCommand>
{
  public:
  DefaultArcadeDriveCommand(Drivetrain *subsystem, std::function<double()> m_forward, std::function<double()> m_rotation);

  void Execute() override;

  private:
  Drivetrain *m_drivetrain;
  std::function<double()> m_forward;
  std::function<double()> m_rotation;
};