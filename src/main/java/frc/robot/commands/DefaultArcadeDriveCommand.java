package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.drivetrain.Drivetrain;

public class DefaultArcadeDriveCommand extends CommandBase {
  private final Drivetrain m_drivetrain;
  private final double m_forward;
  private final double m_rotation;

  public DefaultArcadeDriveCommand(Drivetrain drivetrain, double forward, double rotation) {
    m_drivetrain = drivetrain;
    m_forward = forward;
    m_rotation = rotation;
    addRequirements(drivetrain);
  }

  @Override
  public void execute() {
    m_drivetrain.arcadeDrive(m_forward, m_rotation);
  }
}
