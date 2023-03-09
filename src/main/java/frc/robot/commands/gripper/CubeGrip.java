package frc.robot.commands.gripper;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.RobotContainer;
import frc.robot.subsystems.arm.Arm;

public class CubeGrip extends CommandBase{

  /*Be more precise grabbing a cube, but a similar process to the cone*/
  public CubeGrip(Arm arm){
    addRequirements(arm);
  } 

}
