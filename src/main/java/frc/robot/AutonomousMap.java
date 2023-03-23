package frc.robot;

import java.util.HashMap;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.commands.movement.AutoBalance;
import frc.robot.subsystems.arm.Arm;
import frc.robot.subsystems.drivetrain.Drivetrain;

public class AutonomousMap {

  Arm arm;
  Drivetrain drivetrain;
  TargetMap targetMap;
  static HashMap<String, Command> eventMap = new HashMap<>();

  public AutonomousMap(Drivetrain drivetrain, Arm arm, TargetMap targetMap){
    this.drivetrain = drivetrain;
    this.arm = arm;
    this.targetMap = targetMap;
    eventMap.put("Marker 1", new WaitCommand(3));
    eventMap.put("Reset Position", new InstantCommand(() -> arm.resetPosition()));
    eventMap.put("Auto Balance", new AutoBalance(drivetrain));
    eventMap.put("firstCone3", arm.dropCargo(targetMap.coneArmTargets[2]));
    eventMap.put("firstCube3",  arm.dropCargo(targetMap.cubeArmTargets[2] ));
    
   
  }

}
