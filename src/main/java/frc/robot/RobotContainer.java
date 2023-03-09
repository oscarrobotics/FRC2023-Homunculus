// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import java.util.List;
import com.pathplanner.lib.PathConstraints;
import com.pathplanner.lib.PathPlanner;
import com.pathplanner.lib.PathPlannerTrajectory;
import com.pathplanner.lib.auto.RamseteAutoBuilder;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.CommandGenericHID;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.subsystems.arm.Arm;
import frc.robot.subsystems.drivetrain.Drivetrain;
import io.github.oblarg.oblog.Logger;
import io.github.oblarg.oblog.annotations.Config;
import io.github.oblarg.oblog.annotations.Log;


public class RobotContainer {
  private final CommandXboxController m_driverController = new CommandXboxController(0);

  private final ControllerButtons m_operator = new ControllerButtons(1);

  private final Drivetrain m_drivetrain = new Drivetrain();

  public static final Arm m_arm = new Arm();

  public final AutonomousSelector m_autoSelector = new AutonomousSelector();
  
  private final NetworkTableInstance ntinst = NetworkTableInstance.getDefault();

  final private double k_extrastend = 0;
  private double extrastend = k_extrastend;

  public RobotContainer() {
    

    configureBindings();
    ntinst.startServer();

    
    // m_drivetrain.setDefaultCommand(Commands.run(() -> {
    //   double Smodifier = m_driverController.getRightTriggerAxis() * 0.5 + 0.5-m_driverController.getLeftTriggerAxis()*0.5;
    //   double Tmodifer =1-(( m_driverController.leftBumper().getAsBoolean()?0:1 )* (m_driverController.getRightTriggerAxis()>0.5?1:0)*0.4);
      

    //   m_drivetrain.arcadeDrive(m_driverController.getLeftY() * Smodifier,
    //       m_driverController.getRightX() * Smodifier * Tmodifer);
    // }, m_drivetrain));


    // m_drivetrain.setDefaultCommand(Commands.run(() -> {
    //   double Smodifier = m_driverController.getRightTriggerAxis() * 0.5 + 0.5-m_driverController.getLeftTriggerAxis()*0.5;
    //   // double Tmodifer =1-(( m_driverController.leftBumper().getAsBoolean()?0:1 )* (m_driverController.getRightTriggerAxis()>0.5?1:0)*0.4);
    //   if (m_driverController.getLeftTriggerAxis()>0.5){
    //     m_drivetrain.curvatureDrive(m_driverController.getLeftY() * Smodifier, m_driverController.getRightX()*Smodifier, m_driverController.leftBumper().getAsBoolean());
    //   }else{








      
    //   m_drivetrain.arcadeDrive(m_driverController.getLeftY() * Smodifier,
    //       m_driverController.getRightX() * Smodifier);
    //   }
    // }, m_drivetrain));
    m_drivetrain.setDefaultCommand(Commands.run(() -> {
      double Smodifier = m_driverController.getRightTriggerAxis() * 0.5 + 0.5-m_driverController.getLeftTriggerAxis()*0.42;
      // double Tmodifer =1-(( m_driverController.leftBumper().getAsBoolean()?0:1 )* (m_driverController.getRightTriggerAxis()>0.5 && m_driverController.getLeftTriggerAxis()<0.5?1:0)*0.4); //
      double Tmodifier = (Math.abs( m_driverController.getLeftY())<0.25?Smodifier:0.5);

      m_drivetrain.smoothDrive(m_driverController.getLeftY() * Math.abs(m_driverController.getLeftY())* Smodifier * Constants.maxSpeed,
          m_driverController.getRightX() * Math.abs(m_driverController.getRightX())* Tmodifier * Constants.maxTurn);
    }, m_drivetrain));

    
    m_arm.setDefaultCommand(Commands.run(() -> {
      m_arm.setExtentPosition(m_operator.arcadeBlackLeft().getAsBoolean()?1.2:m_operator.getLeftSlider()+1.3);
      m_arm.setRaisedPosition(m_operator.getRightSlider());
      m_arm.setClawPosition(m_operator.arcadeWhiteLeft().getAsBoolean()?1:-1);
    }, 
    m_arm));

 
   


  }

  private void configureBindings() {
    
  }

  @Log(name = "Get button Configs", tabName = "Buttons")
  public double getSliderConfig(){
    return m_operator.getLeftSlider();
  }

  @Config.NumberSlider(name = "Set left slider")
  public void setLeftSliderConfig(){
    m_operator.getLeftSlider();
  }


  AutonomousMap m_autonMap = new AutonomousMap();
  
  public Command getAutonomousCommand() {
   RamseteAutoBuilder autoBuilder = new RamseteAutoBuilder(
    m_drivetrain::getPose,
    m_drivetrain::resetOdometry,
    m_drivetrain.m_ramseteController,
    m_drivetrain.m_kinematics,
    m_drivetrain::setSpeeds,
    AutonomousMap.eventMap,
    true,
    m_drivetrain
  );   
    Command fullAuto = autoBuilder.fullAuto(m_autoSelector.getSelectedAuto());
    return fullAuto;
  }

  @Config.Command(name = "Reset Position", tabName = "Arm PID")
  Command resetPosition = Commands.run(() -> {
    m_arm.resetPosition();
  }, m_arm);

  
@Log(name = "Extra Stend get", tabName = "Arm PID")
 private double getExtraStend(){
   return extrastend;
 }

 @Config.NumberSlider(name = "Extra Stend set", tabName = "Arm PID", min = 0, max = .4, defaultValue = k_extrastend)
  private void setExtraStend(double value){
    extrastend = value;
  }


}
