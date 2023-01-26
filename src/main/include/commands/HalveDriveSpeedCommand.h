#pragma once

#include <frc2/command/CommandBase.h>
#include <frc2/command/CommandHelper.h>

#include "subsystems/Drivetrain.h"

class HalveDriveSpeedCommand : public frc2::CommandHelper<frc2::CommandBase, HalveDriveSpeedCommand>
{
  public:
  explicit HalveDriveSpeedCommand(Drivetrain *subsystem);

  void Initialize() override;

  void End(bool interrupted) override;

  private:
  Drivetrain *m_drivetrain;
};