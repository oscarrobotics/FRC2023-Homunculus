package frc.robot;

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
  SendableChooser<List<PathPlannerTrajectory>> m_chooser = new SendableChooser<>();
  ArrayList<List<PathPlannerTrajectory>> m_chooser2 = new ArrayList<>();
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
    
    List<PathPlannerTrajectory> BasicAuto = 
          PathPlanner.loadPathGroup(
            "BasicAuto", 
            new PathConstraints(3, 3));
      
    m_chooser.addOption("oneCargoAuto1", oneCargoAuto1);
    m_chooser.addOption("oneCargoAuto2",oneCargoAuto2);
    m_chooser.addOption("oneCargoAuto3",oneCargoAuto3);
    m_chooser.addOption("twoCargoAuto1",twoCargoAuto1);
    m_chooser.addOption("twoCargoAuto2",twoCargoAuto2);
    m_chooser.addOption("twoCargoAuto3",twoCargoAuto3);
    m_chooser.addOption("threeCargoAuto",threeCargoAuto); //NOT RECCOMENDED
    m_chooser.addOption("dropCone3AndLeaveComnityEndAtStation",dropCone3AndLeaveComnityEndAtStation);
    m_chooser.addOption("dropCube3AndLeaveComnityEndAtStation",dropCube3AndLeaveComnityEndAtStation);
    m_chooser.addOption("LeaveComnityEndAtStation",LeaveComnityEndAtStation);
    m_chooser.addOption("BasicAuto",BasicAuto);
    
    m_chooser2.add(oneCargoAuto1);
    m_chooser2.add(oneCargoAuto2);
    m_chooser2.add(oneCargoAuto3);
    m_chooser2.add(twoCargoAuto1);
    m_chooser2.add(twoCargoAuto2);
    m_chooser2.add(twoCargoAuto3);
    m_chooser2.add(threeCargoAuto); //NOT RECCOMENDED
    m_chooser2.add(dropCone3AndLeaveComnityEndAtStation);
    m_chooser2.add(dropCube3AndLeaveComnityEndAtStation);
   m_chooser2.add(LeaveComnityEndAtStation);
   m_chooser2.add(BasicAuto);
  
   autoNames.add("One Cargo Auto 1");
  autoNames.add("One Cargo Auto 2");
  autoNames.add("One Cargo Auto 3");
  autoNames.add("Two Cargo Auto 1");
  autoNames.add("Two Cargo Auto 2");
  autoNames.add("Two Cargo Auto 3");
  autoNames.add("Three Cargo Auto");
  autoNames.add("Drop Cone 3 and Leave Community End at Station");
  autoNames.add("Drop Cube 3 and Leave Community End at Station");
  autoNames.add("Leave Community End at Station");
  autoNames.add("BasicAuto");
  SmartDashboard.putData("choose auto path" ,m_chooser);

  }

  public int getSelectedAuto() {
    return m_autoSelected;
  }

  public List<PathPlannerTrajectory> getAutoPath(int i) {
    return m_chooser2.get(i);
  }


  @Config.NumberSlider(name = "Auto Selectcor", min = -1, max = 6, blockIncrement = 1 ,defaultValue = -1, tabName = "Target Selector", columnIndex = 1, rowIndex = 4) 
  double setauto1(double auto1){
     return m_autoSelected = (int)auto1;
  }

  @Log.ToString(name = "Selected Auto" , tabName = "Target Selector", columnIndex = 4, rowIndex = 4)
  String getSelectedAutoName(){
    if (m_autoSelected == -1) {
      return "Do Nothing";
    }
    return autoNames.get(m_autoSelected);
  }




  }



