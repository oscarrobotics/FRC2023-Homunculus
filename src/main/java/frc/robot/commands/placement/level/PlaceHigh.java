package frc.robot.commands.placement.level;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.arm.Arm;

public class PlaceHigh extends CommandBase{

  Arm arm;
  double initExtPos, initRaisePos, initGripPos;
  double extendError, raiseError;
  public PlaceHigh(Arm arm){
    this.arm = arm;
    addRequirements(arm);
  }

  @Override
  public void initialize(){
    initExtPos = arm.getExtendedPosition();
    initRaisePos = arm.getRaisedPosition();
    initGripPos = arm.getGripPosition();
  }

  @Override
  public void execute(){
    extendError = (arm.k_targetExtPosHigh - initExtPos);
    
  }
}