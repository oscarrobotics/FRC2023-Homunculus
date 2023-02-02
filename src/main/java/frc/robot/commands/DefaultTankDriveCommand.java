package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.drivetrain.Drivetrain;

public class DefaultTankDriveCommand extends CommandBase {
 private final Drivetrain m_drivetrain;
  private final double m_leftSpeed;
  private final double m_rightSpeed;
  
  public DefaultTankDriveCommand(Drivetrain drivetrain, double leftSpeed, double rightSpeed) {
    m_drivetrain = drivetrain;
    m_leftSpeed = leftSpeed;
    m_rightSpeed = rightSpeed;
    addRequirements(drivetrain);
  }

  @Override
  public void execute() {
    m_drivetrain.tankDrive(m_leftSpeed, m_rightSpeed);
  }
}
