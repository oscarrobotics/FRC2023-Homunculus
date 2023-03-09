package frc.robot.commands.arm;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.arm.Arm;

public class ExtendArm extends CommandBase{
  public ExtendArm(Arm arm){
    addRequirements(arm);
  }

  @Override
  public void initialize() {
    // TODO Auto-generated method stub
    
  }

  @Override
  public void execute() {
    // TODO Auto-generated method stub
    
  }

  @Override
  public void end(boolean interrupted) {
    // TODO Auto-generated method stub
    
  }

  @Override
  public boolean isFinished() {
    // TODO Auto-generated method stub
    return false;
  }
}
