#pragma once

#include <frc2/command/SubsystemBase.h>

class Vision : public frc2::SubsystemBase
{
  public:
  Vision();

  void Periodic() override;
  void SimulationPeriodic() override;

  private:
};