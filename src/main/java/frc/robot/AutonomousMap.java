package frc.robot;

import java.util.HashMap;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import frc.robot.commands.movement.AutoBalance;
import frc.robot.subsystems.arm.Arm;
import frc.robot.subsystems.drivetrain.Drivetrain;

public class AutonomousMap {

  Arm m_arm = new Arm();
  Drivetrain m_Drivetrain = new Drivetrain();
  static HashMap<String, Command> eventMap = new HashMap<>();

  public AutonomousMap(){
    eventMap.put("Marker 1", new WaitCommand(3));
    eventMap.put("Reset Position", new InstantCommand(() -> m_arm.resetPosition()));
    eventMap.put("Auto Balance", new AutoBalance(m_Drivetrain));
  }

}
