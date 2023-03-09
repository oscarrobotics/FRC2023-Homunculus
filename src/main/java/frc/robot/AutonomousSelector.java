package frc.robot;

import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj2.command.Command;
import java.util.List;
import com.pathplanner.lib.PathConstraints;
import com.pathplanner.lib.PathPlanner;
import com.pathplanner.lib.PathPlannerTrajectory;
public class AutonomousSelector {

  public AutonomousSelector(){
    List<PathPlannerTrajectory> oneCargoAuto1 = 
        PathPlanner.loadPathGroup(
          "OneCargoAuto1", 
          new PathConstraints(3, 4));
    List<PathPlannerTrajectory> oneCargoAuto2 = 
        PathPlanner.loadPathGroup(
          "OneCargoAuto2", 
          new PathConstraints(3, 4));
    List<PathPlannerTrajectory> oneCargoAuto3 = 
        PathPlanner.loadPathGroup(
          "OneCargoAuto3", 
          new PathConstraints(3, 4));
    List<PathPlannerTrajectory> twoCargoAuto1 = 
        PathPlanner.loadPathGroup(
          "TwoCargoAuto1", 
          new PathConstraints(3, 4));
    List<PathPlannerTrajectory> twoCargoAuto2 = 
        PathPlanner.loadPathGroup(
          "TwoCargoAuto2", 
          new PathConstraints(3, 4));
    List<PathPlannerTrajectory> twoCargoAuto3 = 
        PathPlanner.loadPathGroup(
          "TwoCargoAuto3", 
          new PathConstraints(3, 4));
    List<PathPlannerTrajectory> threeCargoAuto = //certainly don't try this first comp
        PathPlanner.loadPathGroup(
          "ThreeCargoAuto", 
          new PathConstraints(3, 4));
    
    SendableChooser<Command> m_chooser = new SendableChooser<>();
    
  }
}
