#pragma once

#include <frc2/command/CommandBase.h>
#include <frc2/command/CommandHelper.h>

#include "subsystems/Drivetrain.h"

class UncapDriveSpeedCommand : public frc2::CommandHelper<frc2::CommandBase, UncapDriveSpeedCommand>
{
  public:
  explicit UncapDriveSpeedCommand(Drivetrain *subsystem);

  void Initialize() override;

  void End(bool interrupted) override;

  private:
  Drivetrain *m_drivetrain;
};