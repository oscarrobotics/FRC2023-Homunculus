#pragma once

#include <frc2/command/CommandBase.h>
#include <frc2/command/CommandHelper.h>

#include "subsystems/Drivetrain.h"

class DefaultTankDriveCommand : public frc2::CommandHelper<frc2::CommandBase, DefaultTankDriveCommand>
{
  public:
  DefaultTankDriveCommand(Drivetrain *subsystem, std::function<double()> left, std::function<double()> right);

  void Execute() override;

  private:
  Drivetrain *m_drivetrain;
  std::function<double()> m_left;
  std::function<double()> m_right;
};