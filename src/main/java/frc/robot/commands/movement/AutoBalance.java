package frc.robot.commands.movement;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.RobotContainer;
import frc.robot.subsystems.drivetrain.Drivetrain;
import io.github.oblarg.oblog.Loggable;
import io.github.oblarg.oblog.annotations.Log;

public class AutoBalance extends CommandBase implements Loggable{

  private boolean m_reverse = false;
  private boolean isBalancing = false;
  private boolean done = false;
  private double targetYaw;
  private double rampDeg = 14;
  private double reverseFactor = 1;
  private double forwardPower= 0.0; //TODO: figure out this value

  Drivetrain m_drivetrain;
  Timer endTimer = new Timer();

  public AutoBalance(Drivetrain drive, boolean reverse) {
    // Use addRequirements() here to declare subsystem dependencies.
    m_drivetrain = drive;
    addRequirements(drive);
    m_reverse = reverse;
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    isBalancing = false;
    done = false;
    targetYaw = 180;
    if(m_reverse){
      reverseFactor = -1;
    }
    m_drivetrain.smoothDrive(3 * reverseFactor, 0);
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
      double pitch = m_drivetrain.getGyroPitch();

      if(pitch > rampDeg && !isBalancing){
        m_drivetrain.smoothDrive(0, 0);
        isBalancing = false;
        done = true;
      }
      if(isBalancing){
        if(m_reverse && Math.abs(pitch) < 4.5){
          m_drivetrain.smoothDrive(0, 0);
          isBalancing = false;
          done = true;
        }
        if(!m_reverse && Math.abs(pitch) < 4.5 ){
          m_drivetrain.smoothDrive(0.6, 0);
          m_drivetrain.smoothDrive(0, 0);
          isBalancing = false;
          done = true;
        }
        else {
          double powerSign = pitch > 0 ? 1 : -1;
          double maxPitch = 30;

          double percentage = MathUtil.clamp(Math.abs(pitch) / maxPitch, 0.0, 0.65) * forwardPower;
      }
    }

    // endTimer.start();
    // Double currentAngle = m_drivetrain.getGyroRoll();
    // Double angleDifference = currentAngle - lastAngle;
    // if (angleDifference > 2){
    //   m_drivetrain.arcadeDriveV(0.5,0);
    // } else if (angleDifference < 2){
    //   m_drivetrain.arcadeDriveV(-0.5, 0);
    // } else {
    //   m_drivetrain.arcadeDriveV(0, 0);
    // }
    
    // lastAngle = m_drivetrain.getGyroRoll();
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    // m_drivetrain.m_leftMotors.stopMotor();
    // m_drivetrain.m_rightMotors.stopMotor();
  }


  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    boolean finished = (m_drivetrain.getGyroRoll() < 2 && m_drivetrain.getGyroRoll() > -2 && endTimer.get() > 1? true: false);
    return finished;
  }
}
