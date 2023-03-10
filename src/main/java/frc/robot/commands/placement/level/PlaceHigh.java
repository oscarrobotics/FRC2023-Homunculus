package frc.robot.commands.placement.level;

import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.ParallelRaceGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import edu.wpi.first.wpilibj2.command.WaitUntilCommand;
import frc.robot.subsystems.arm.Arm;

public class PlaceHigh extends SequentialCommandGroup{

  Arm arm;
  double initExtPos, initRaisePos, initGripPos;
  double extendError, raiseError;
  public PlaceHigh(Arm arm){
    this.arm = arm;
    addRequirements(arm);

    initExtPos = arm.getExtendedPosition();
    initRaisePos = arm.getRaisedPosition();
    initGripPos = arm.getGripPosition();

    extendError = (arm.k_targetExtPosHigh - initExtPos);
    raiseError = (arm.k_targetRaisePosHigh - initRaisePos);

    addCommands(
      new SequentialCommandGroup(
        new ParallelRaceGroup(
          new InstantCommand(() -> arm.setExtentPosition(initExtPos + extendError * 0.5)),
          new InstantCommand(() -> arm.setRaisedPosition(initRaisePos + raiseError * 0.5)),
          new InstantCommand(() -> arm.setClawPosition(initGripPos + arm.k_targetGripPosHigh - initGripPos)),
          new WaitCommand(0.5),
          new InstantCommand(() -> arm.setClawPosition(-0.8)),
          new InstantCommand(() -> arm.setRaisedPosition(0)),
          new InstantCommand(() -> arm.setClawPosition(0)))
          )
        ); //,
        // new ParallelRaceGroup(
        //   new InstantCommand(() -> arm.setExtendedPosition(initExtPos + extendError * 0.75)),
        //   new InstantCommand(() -> arm.setRaisedPosition(initRaisePos + raiseError * 0.75)),
        //   new InstantCommand(() -> arm.setGripPosition(initGripPos + arm.k_targetGripPosHigh - initGripPos)),
        //   new WaitCommand(0.5)
        // ),
        // new ParallelRaceGroup(
        //   new InstantCommand(() -> arm.setExtendedPosition(initExtPos + extendError)),
        //   new InstantCommand(() -> arm.setRaisedPosition(initRaisePos + raiseError)),
        //   new InstantCommand(() -> arm.setGripPosition(initGripPos + arm.k_targetGripPosHigh - initGripPos)),
        //   new WaitCommand(0.5)
  }

  // @Override
  // public void end(boolean interrupted) {
    
  // }

}