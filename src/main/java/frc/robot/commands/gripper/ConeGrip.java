package frc.robot.commands.gripper;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.RobotContainer;
import frc.robot.subsystems.arm.Arm;

public class ConeGrip extends CommandBase{

  /*Just grab as much as possible for the cone */
  public ConeGrip(Arm arm){
    addRequirements(arm);
  } 

}
