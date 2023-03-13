package frc.robot;

import java.util.HashMap;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.subsystems.arm.Arm;
import frc.robot.subsystems.drivetrain.Drivetrain;

public class AutonomousMap {

  Arm arm;
  Drivetrain drivetrain;
  static HashMap<String, Command> eventMap = new HashMap<>();

  public AutonomousMap(Drivetrain drivetrain, Arm arm){
    this.drivetrain = drivetrain;
    this.arm = arm;
    eventMap.put("Marker 1", new WaitCommand(3));
    // eventMap.put("Reset Position", new InstantCommand(() -> arm.resetPosition()));
    // eventMap.put("Auto Balance", new AutoBalance(drivetrain));  
   
  }

}
