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
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
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

  public final TargetSelector m_targetSelector = new TargetSelector();
  
  private final NetworkTableInstance ntinst = NetworkTableInstance.getDefault();



  final private double k_extrastend = 0;
  private double extrastend = k_extrastend;
  Boolean safe = false;
    Boolean motion = false;

  public RobotContainer() {
    

    configureBindings();
    ntinst.startServer();

    
    // m_drivetrain.setDefaultCommand(new InstantCommand (() -> {
    //   double Smodifier = m_driverController.getRightTriggerAxis() * 0.5 + 0.5-m_driverController.getLeftTriggerAxis()*0.5;
    //   double Tmodifer =1-(( m_driverController.leftBumper().getAsBoolean()?0:1 )* (m_driverController.getRightTriggerAxis()>0.5?1:0)*0.4);
      

    //   m_drivetrain.arcadeDrive(m_driverController.getLeftY() * Smodifier,
    //       m_driverController.getRightX() * Smodifier * Tmodifer);
    // }, m_drivetrain));


    // m_drivetrain.setDefaultCommand(new InstantCommand (() -> {
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

      

    m_operator.arcadeBlackLeft().onTrue(new InstantCommand(()->{
      safe = true;
      motion = false;

    }));
    m_operator.arcadeBlackLeft().onFalse(new InstantCommand(()->{
      safe = false;
      motion = false;

    }));
    m_operator.arcadeBlackRight().onTrue(new InstantCommand(()->{
      safe = false;
      motion = true;

    }));
    m_operator.arcadeBlackRight().onFalse(new InstantCommand(()->{
      safe = false;
      motion = false;

    }));


    if(!safe && !motion){  
      m_arm.setDefaultCommand(Commands.run(() -> {
        m_arm.setExtendPosition(m_operator.getLeftSlider());
        m_arm.setRaisePosition(-m_operator.getRightSlider());
        // m_arm.setClawPosition(m_operator.arcadeWhiteLeft().getAsBoolean()?1:-1);
      }, 
      m_arm));
  }
  m_operator.arcadeWhiteLeft().onTrue(new InstantCommand(()->{m_arm.toggleGrip();}));
  
  if(safe && !motion){  
    m_arm.setDefaultCommand(Commands.run(() -> {
      m_arm.setExtendPositionSafe(m_operator.getLeftSlider());
      m_arm.setRaisePositionSafe(-m_operator.getRightSlider());
      m_arm.setClawPosition(m_operator.arcadeWhiteLeft().getAsBoolean()?1:-1);
    }, 
    m_arm));
}
if(!safe && motion){
    m_arm.setDefaultCommand(Commands.run(() -> {
      m_arm.setExtendMotion(m_operator.getLeftSlider());
      m_arm.setRaiseMotion(-m_operator.getRightSlider());
      m_arm.setClawMotion(m_operator.arcadeWhiteLeft().getAsBoolean()?1:-1);
    }, 
    m_arm));
    }
    //set driver buttons
    // m_driverController.x().whileTrue(m_drivetrain.goToPoseCommand(
    //   TargetMap.getTargetPose( TargetSelector.getTargetIdx()), 
    //   3, 3));
    // m_driverController.y().whileTrue(m_drivetrain.goToPoseCommand(TargetMap.getStationPose(0),3,3));
    // m_driverController.b().whileTrue(m_drivetrain.goToPoseCommand(TargetMap.getStationPose(1),3,3));
    // m_driverController.a().whileTrue(m_drivetrain.goToPoseCommand(TargetMap.getStationPose(2),3,3));
      

    // //set the operator buttons for the arm
    // //sets the arm to the currently selected target
    // m_operator.arcadeBlackRight().whileTrue(new InstantCommand(()->{
    //   m_arm.setArmPosition(TargetMap.getArmTarget(TargetSelector.getTargetIdx()));
    // }, m_arm));
    // //sets the arm to the target dual station
    // m_operator.arcadeWhiteRight().whileTrue(new InstantCommand(()->{
    //   m_arm.setArmPosition(TargetMap.stationArmTargets[0]);
    // }, m_arm));

    //set the butons on the strat conm to select the target for autoalignment, NOT for autonomous
   m_operator.sc1().onTrue( new InstantCommand (()->{TargetSelector.setA();}));
    m_operator.sc2().onTrue(new InstantCommand (()->{TargetSelector.setB();}));
    m_operator.sc3().onTrue(new InstantCommand (()->{TargetSelector.setC();}));

    m_operator.sc4().onTrue(new InstantCommand (()->{TargetSelector.setLeft();}));
    m_operator.sc5().onTrue(new InstantCommand (()->{TargetSelector.setMid();}));
    m_operator.sc6().onTrue(new InstantCommand (()->{TargetSelector.setRight();}));

    m_operator.scSideTop().onTrue(new InstantCommand (()->{TargetSelector.setBack();}));
    m_operator.scSideMid().onTrue(new InstantCommand (()->{TargetSelector.setMid();}));
    m_operator.scSideBot().onTrue(new InstantCommand (()->{TargetSelector.setFront();}));


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


  AutonomousMap m_autonMap = new AutonomousMap(m_drivetrain, m_arm);
  
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

  Command fullAuto ;
    if(m_autoSelector.getSelectedAuto() >=0 ){
      fullAuto = autoBuilder.fullAuto(m_autoSelector.getAutoPath(m_autoSelector.getSelectedAuto())) ;
    }
    else{
      fullAuto = new WaitCommand(2);
    }
    return fullAuto;
    
  }
   public Command getAutonomousCommand2() {
    //cammad drive backwards 3 meters wait 2 seconds then drive forward 1.2 meters then back .2 meters using just velocity control
    return new SequentialCommandGroup(
     new InstantCommand(()->m_drivetrain.setSpeedsCONT(-1, -1)).withTimeout(2),
     new WaitCommand(2),
      new InstantCommand(()->m_drivetrain.setSpeedsCONT(1, 1)).withTimeout(1.2),
      new WaitCommand(0.5),
      new InstantCommand(()->m_drivetrain.setSpeedsCONT(-1, -1)).withTimeout(.2),
      new InstantCommand(()->m_drivetrain.setSpeedsCONT(0, 0)).withTimeout(5));

   }
 
   
    


   
  


  @Config.Command(name = "Reset Position", tabName = "Arm PID")
  InstantCommand resetPosition = new InstantCommand(() -> {
    m_arm.resetPosition();
  }, m_arm);

  
@Log(name = "Extra Stend get", tabName = "Arm PID")
 private double getExtraStend(){
   return extrastend;
 }

 @Config.NumberSlider(name = "Extra Stend set", tabName = "Arm PID", min = 0, max = .2, defaultValue = k_extrastend)
  private void setExtraStend(double value){
    extrastend = value;
  }


}
