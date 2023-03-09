package frc.robot;

import java.util.HashMap;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import edu.wpi.first.wpilibj2.command.Command;
import com.pathplanner.lib.auto.SwerveAutoBuilder;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import frc.robot.subsystems.drivetrain.Drivetrain;

public class AutonomousMap {

  static HashMap<String, Command> eventMap = new HashMap<>();
  public AutonomousMap(){
    eventMap.put("Marker 1", new WaitCommand(3));
    
  }

}
