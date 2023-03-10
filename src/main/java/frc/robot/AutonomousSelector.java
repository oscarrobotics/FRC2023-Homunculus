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
public class AutonomousSelector extends SubsystemBase implements Loggable{

  // SendableChooser<List <PathPlannerTrajectory>> m_chooser;

  int m_autoSelected;
  ArrayList<List<PathPlannerTrajectory>> autoPaths = new ArrayList<List<PathPlannerTrajectory>>();
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
    
    autoPaths.add(oneCargoAuto1);
    autoPaths.add(oneCargoAuto2);
    autoPaths.add(oneCargoAuto3);
    autoPaths.add(twoCargoAuto1);
    autoPaths.add(twoCargoAuto2);
    autoPaths.add(twoCargoAuto3);
    autoPaths.add(threeCargoAuto);

    autoNames.add("One Cargo Auto 1");
    autoNames.add("One Cargo Auto 2");
    autoNames.add("One Cargo Auto 3");
    autoNames.add("Two Cargo Auto 1");
    autoNames.add("Two Cargo Auto 2");
    autoNames.add("Two Cargo Auto 3");
    autoNames.add("Three Cargo Auto");


    // m_chooser = new SendableChooser<>();
    // m_chooser.setDefaultOption("One Cargo Auto 1", oneCargoAuto1);
    // // m_chooser.addOption("One Cargo Auto 2", oneCargoAuto2);
    // // m_chooser.addOption("One Cargo Auto 3", oneCargoAuto3);
    // // m_chooser.addOption("Two Cargo Auto 1", twoCargoAuto1);
    // // m_chooser.addOption("Two Cargo Auto 2", twoCargoAuto2);
    // // m_chooser.addOption("Two Cargo Auto 3", twoCargoAuto3);
    // // m_chooser.addOption("Three Cargo Auto", threeCargoAuto); //NOT RECCOMENDED
    
    // SmartDashboard.putData(m_chooser);

  }
  
  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    
  }

  public int getSelectedAuto() {
    return m_autoSelected;
  }

  public List<PathPlannerTrajectory> getAutoPath(int i) {
    return autoPaths.get(i);
  }


  @Config.NumberSlider(name = "Auto Selectcor", min = -1, max = 6, blockIncrement = 1 ,defaultValue = -1)
  void setauto1(double auto1){
    m_autoSelected = (int)auto1;
  }

  @Log.ToString(name = "Selected Auto")
  String getSelectedAutoName(){
    return autoNames.get(m_autoSelected);
  }




  }



