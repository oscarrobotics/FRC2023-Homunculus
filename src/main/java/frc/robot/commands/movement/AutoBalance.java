package frc.robot.commands.movement;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.RobotContainer;
import frc.robot.subsystems.drivetrain.Drivetrain;
import io.github.oblarg.oblog.Loggable;
import io.github.oblarg.oblog.annotations.Log;

public class AutoBalance extends CommandBase implements Loggable{
  
  Drivetrain m_drivetrain = new Drivetrain();
  Double lastAngle;
  Timer endTimer; 
  public AutoBalance(Drivetrain drive) {
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(drive);
    endTimer = new Timer();
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    lastAngle = m_drivetrain.getGyroRoll();
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    endTimer.start();
    Double currentAngle = m_drivetrain.getGyroRoll();
    Double angleDifference = currentAngle - lastAngle;
    if (angleDifference > 2){
      m_drivetrain.arcadeDriveV(0.5,0);
    } else if (angleDifference < 2){
      m_drivetrain.arcadeDriveV(-0.5, 0);
    } else {
      m_drivetrain.arcadeDriveV(0, 0);
    }
    
    lastAngle = m_drivetrain.getGyroRoll();
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    m_drivetrain.m_leftMotors.stopMotor();
    m_drivetrain.m_rightMotors.stopMotor();
  }


  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    boolean finished = (m_drivetrain.getGyroRoll() < 2 && m_drivetrain.getGyroRoll() > -2 && endTimer.get() > 1? true: false);
    return finished;
  }
}
