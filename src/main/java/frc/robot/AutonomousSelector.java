package frc.robot;

import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import io.github.oblarg.oblog.Loggable;
import io.github.oblarg.oblog.annotations.Config;
import io.github.oblarg.oblog.annotations.Log;
import io.github.oblarg.oblog.annotations.Log.Field2d;
import java.util.ArrayList;
import java.util.List;
import com.pathplanner.lib.PathConstraints;
import com.pathplanner.lib.PathPlanner;
import com.pathplanner.lib.PathPlannerTrajectory;
public class AutonomousSelector implements Loggable{

  // SendableChooser<List <PathPlannerTrajectory>> m_chooser;

  int m_autoSelected;
  //ArrayList<List<PathPlannerTrajectory>> m_chooser = new ArrayList<List<PathPlannerTrajectory>>();
  SendableChooser<List<PathPlannerTrajectory>> m_chooser = new SendableChooser<>();
  ArrayList<String> autoNames = new ArrayList<String>();
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
    List<PathPlannerTrajectory> dropCone3AndLeaveComnityEndAtStation = 
        PathPlanner.loadPathGroup(
          "dropCone3AndLeaveComnityEndAtStation", 
          new PathConstraints(3, 3));
    List<PathPlannerTrajectory> dropCube3AndLeaveComnityEndAtStation = 
        PathPlanner.loadPathGroup(
          "dropCube3AndLeaveComnityEndAtStation", 
          new PathConstraints(3, 3));
      List<PathPlannerTrajectory> LeaveComnityEndAtStation = 
        PathPlanner.loadPathGroup(
          "LeaveComnityEndAtStation", 
          new PathConstraints(3, 3));
    

    m_chooser.addOption("One Cargo Auto 1", oneCargoAuto1);

    m_chooser.addOption("One Cargo Auto 2", oneCargoAuto2);
    m_chooser.addOption("One Cargo Auto 3", oneCargoAuto3);
    m_chooser.addOption("Two Cargo Auto 1", twoCargoAuto1);
    m_chooser.addOption("Two Cargo Auto 2",twoCargoAuto2);
    m_chooser.addOption("Two Cargo Auto 3",twoCargoAuto3);
    m_chooser.addOption("Three Cargo Auto", threeCargoAuto); //NOT RECCOMENDED
    m_chooser.addOption("Drop Cone 3 and Leave Community, End at Station", dropCone3AndLeaveComnityEndAtStation);
    m_chooser.addOption("Drop Cube 3 and Leave Community, End at Station", dropCube3AndLeaveComnityEndAtStation);
   m_chooser.addOption("Leave Community + End At Station", LeaveComnityEndAtStation);
  
  //  autoNames.add("One Cargo Auto 1");
  // autoNames.add("One Cargo Auto 2");
  // autoNames.add("One Cargo Auto 3");
  // autoNames.add("Two Cargo Auto 1");
  // autoNames.add("Two Cargo Auto 2");
  // autoNames.add("Two Cargo Auto 3");
  // autoNames.add("Three Cargo Auto");
  // autoNames.add("Drop Cone 3 and Leave Community End at Station");
  // autoNames.add("Drop Cube 3 and Leave Community End at Station");
  // autoNames.add("Leave Community End at Station");

    SmartDashboard.putData("Choose auto path", m_chooser);
  }

  public int getSelectedAuto() {
    return m_autoSelected;
  }

  public List<PathPlannerTrajectory> getAutoPath() {
    return m_chooser.getSelected();
  }


  @Config.NumberSlider(name = "Auto Selectcor", min = -1, max = 6, blockIncrement = 1 ,defaultValue = -1, tabName = "Target Selector", columnIndex = 1, rowIndex = 4) 
  double setauto1(double auto1){
     return m_autoSelected = (int)auto1;
  }

  // @Log.ToString(name = "Selected Auto" , tabName = "Target Selector", columnIndex = 4, rowIndex = 4)
  // String getSelectedAutoName(){
  //   if (m_autoSelected == -1) {
  //     return "Do Nothing";
  //   }
  //   return autoNames.get(m_autoSelected);
  // }

  }