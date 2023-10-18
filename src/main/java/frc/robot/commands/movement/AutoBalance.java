package frc.robot.commands.movement;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.subsystems.drivetrain.Drivetrain;
import io.github.oblarg.oblog.Loggable;

public class AutoBalance extends SequentialCommandGroup implements Loggable {

  private final double m_rateThreshold = 5;
  private final double m_climbRateTimeout = 1.3;

  private double m_climbingSign = 0.0;
  private final Timer m_timer = new Timer();

  private final double m_angleThreshold = 14; //change to 14 degs during practice

  public AutoBalance(Drivetrain m_drivetrain){

    CommandBase getOnDriveStation = Commands.run(
      ()-> m_drivetrain.smoothDrive(1.2,0), m_drivetrain)
      .until(()-> Math.abs(m_drivetrain.getGyroPitch()) > m_angleThreshold)
      .finallyDo((intr)->{
        m_climbingSign = Math.signum(m_drivetrain.getGyroPitch());
      });

      CommandBase adjustPos = Commands.run(() -> {
          m_drivetrain.smoothDrive(1, 0.0);}, m_drivetrain)
          .until(() -> {
            var curPitchRate = m_drivetrain.getFilteredGyroPitchRate();
            System.out.println(m_climbingSign);
            System.out.println(curPitchRate);
            if(m_climbingSign == 1){
              if(curPitchRate > m_rateThreshold){
                return m_timer.hasElapsed(m_climbRateTimeout);
              }
            }
            else {
              if(curPitchRate < m_rateThreshold * -1){
                return m_timer.hasElapsed(m_climbRateTimeout);
              }
            }
            return false;
        
          });
          CommandBase adjustPosF = Commands.runOnce(
             ()->{m_timer.restart();}     )
            .andThen(new RunCommand(() -> {
            m_drivetrain.smoothDrive(-0.7, 0.0);
            }, m_drivetrain)).until(()->{return m_timer.hasElapsed(0.75);})
      
            .andThen(() -> {
              m_drivetrain.smoothDrive(0, 0.0);}, m_drivetrain);
           
          
          
      addCommands(
          getOnDriveStation,
          Commands.runOnce(() -> {
            m_timer.restart();
          }),
          adjustPos
          , adjustPosF
          );
    }
}