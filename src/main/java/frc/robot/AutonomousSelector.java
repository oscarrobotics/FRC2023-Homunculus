package frc.robot;

import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import io.github.oblarg.oblog.annotations.Config;

import java.util.List;
import com.pathplanner.lib.PathConstraints;
import com.pathplanner.lib.PathPlanner;
import com.pathplanner.lib.PathPlannerTrajectory;
public class AutonomousSelector extends SubsystemBase{

  SendableChooser<List <PathPlannerTrajectory>> m_chooser;

  public AutonomousSelector(){
    List<PathPlannerTrajectory> oneCargoAuto1 = 
        PathPlanner.loadPathGroup(
          "OneCargoAuto1", 
          new PathConstraints(3, 3));
    List<PathPlannerTrajectory> oneCargoAuto2 = 
        PathPlanner.loadPathGroup(
          "OneCargoAuto2", 
          new PathConstraints(3, 3));
    List<PathPlannerTrajectory> oneCargoAuto3 = 
        PathPlanner.loadPathGroup(
          "OneCargoAuto3", 
          new PathConstraints(3, 3));
    List<PathPlannerTrajectory> twoCargoAuto1 = 
        PathPlanner.loadPathGroup(
          "TwoCargoAuto1", 
          new PathConstraints(3, 3));
    List<PathPlannerTrajectory> twoCargoAuto2 = 
        PathPlanner.loadPathGroup(
          "TwoCargoAuto2", 
          new PathConstraints(3, 3));
    List<PathPlannerTrajectory> twoCargoAuto3 = 
        PathPlanner.loadPathGroup(
          "TwoCargoAuto3", 
          new PathConstraints(3, 3));
    List<PathPlannerTrajectory> threeCargoAuto = //certainly don't try this first comp
        PathPlanner.loadPathGroup(
          "ThreeCargoAuto", 
          new PathConstraints(3, 3));
    
    m_chooser = new SendableChooser<>();
    m_chooser.setDefaultOption("One Cargo Auto 1", oneCargoAuto1);
    m_chooser.addOption("One Cargo Auto 2", oneCargoAuto2);
    m_chooser.addOption("One Cargo Auto 3", oneCargoAuto3);
    m_chooser.addOption("Two Cargo Auto 1", twoCargoAuto1);
    m_chooser.addOption("Two Cargo Auto 2", twoCargoAuto2);
    m_chooser.addOption("Two Cargo Auto 3", twoCargoAuto3);
    m_chooser.addOption("Three Cargo Auto", threeCargoAuto); //NOT RECCOMENDED
    
    SmartDashboard.putData(m_chooser);

  }
  
  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    
  }

  @Config(name = "Auton selector", tabName = "Autonomous")
  public List<PathPlannerTrajectory> getSelectedAuto() {
    return m_chooser.getSelected();
  }

}
