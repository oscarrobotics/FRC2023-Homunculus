// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import java.util.List;

import com.fasterxml.jackson.databind.deser.DataFormatReaders.Match;
import com.pathplanner.lib.PathConstraints;
import com.pathplanner.lib.PathPlanner;
import com.pathplanner.lib.PathPlannerTrajectory;
import com.pathplanner.lib.auto.RamseteAutoBuilder;

import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.interfaces.Gyro;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.ScheduleCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import edu.wpi.first.wpilibj2.command.button.CommandGenericHID;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.commands.movement.AutoBalance;
// import frc.robot.commands.movement.AutoBalance;
import frc.robot.subsystems.arm.Arm;
import frc.robot.subsystems.drivetrain.Drivetrain;
import io.github.oblarg.oblog.Loggable;
import io.github.oblarg.oblog.Logger;
import io.github.oblarg.oblog.annotations.Config;
import io.github.oblarg.oblog.annotations.Log;



public class RobotContainer implements Loggable{
  
  private final CommandXboxController m_driverController = new CommandXboxController(0);

  private final ControllerButtons m_operator = new ControllerButtons(1);

  private final Drivetrain m_drivetrain = new Drivetrain();

  public static final Arm m_arm = new Arm();

  public static final TargetMap m_targetMap = new TargetMap();

  public static final TargetSelector m_targetSelector = new TargetSelector();


  // public final PathOptions m_autoSelector = new PathOptions();

  private final NetworkTableInstance ntinst = NetworkTableInstance.getDefault();



  final private double k_extrastend = 0;
  private double extrastend = k_extrastend;
  Boolean safe = false;
  Boolean motion = false;

  double move = 0;

  public RobotContainer() {


    configureBindings();
    ntinst.startServer();
    


    // m_drivetrain.setDefaultCommand(new InstantCommand (() -> {
    // double Smodifier = m_driverController.getRightTriggerAxis() * 0.5 +
    // 0.5-m_driverController.getLeftTriggerAxis()*0.5;
    // double Tmodifer =1-(( m_driverController.leftBumper().getAsBoolean()?0:1 )*
    // (m_driverController.getRightTriggerAxis()>0.5?1:0)*0.4);


    // m_drivetrain.arcadeDrive(m_driverController.getLeftY() * Smodifier,
    // m_driverController.getRightX() * Smodifier * Tmodifer);
    // }, m_drivetrain));


    // m_drivetrain.setDefaultCommand(new InstantCommand (() -> {
    // double Smodifier = m_driverController.getRightTriggerAxis() * 0.5 +
    // 0.5-m_driverController.getLeftTriggerAxis()*0.5;
    // // double Tmodifer =1-(( m_driverController.leftBumper().getAsBoolean()?0:1 )*
    // (m_driverController.getRightTriggerAxis()>0.5?1:0)*0.4);
    // if (m_driverController.getLeftTriggerAxis()>0.5){
    // m_drivetrain.curvatureDrive(m_driverController.getLeftY() * Smodifier,
    // m_driverController.getRightX()*Smodifier, m_driverController.leftBumper().getAsBoolean());
    // }else{



    // m_drivetrain.arcadeDrive(m_driverController.getLeftY() * Smodifier,
    // m_driverController.getRightX() * Smodifier);
    // }
    // }, m_drivetrain));
    m_drivetrain.setDefaultCommand(Commands.run(() -> {
      double Smodifier = m_driverController.getRightTriggerAxis() * 0.5 + 0.5
          - m_driverController.getLeftTriggerAxis() * 0.30;
      // double Tmodifer =1-(( m_driverController.leftBumper().getAsBoolean()?0:1 )*
      // (m_driverController.getRightTriggerAxis()>0.5 &&
      // m_driverController.getLeftTriggerAxis()<0.5?1:0)*0.4); //
      double Tmodifier =
          (Math.abs(m_driverController.getLeftY()) < 0.18
           ? Smodifier *1.4: Math.min(0.5, Smodifier));

      m_drivetrain.smoothDrive(
          -1 * m_driverController.getLeftY() * Math.abs(m_driverController.getLeftY()) * Math.abs(m_driverController.getLeftY())* Smodifier
              * Constants.maxSpeed,
          m_driverController.getRightX() * -1 * Math.abs(m_driverController.getRightX())* Tmodifier
              * Constants.maxTurn);
    }, m_drivetrain));



 
      // m_arm.setDefaultCommand(Commands.run(() -> {
      //   m_arm.setExtendPositionArbFF(m_operator.getLeftSlider());
      //   m_arm.setRaisePositionArbFF(-m_operator.getRightSlider());
      //   m_arm.setGripSpeed(300);
       
      // }, m_arm));


      
      m_arm.setDefaultCommand(Commands.run(() -> {
        m_arm.setArmPositionSafe(-m_operator.getRightSlider(), m_operator.getLeftSlider()) ;
        // m_arm.setGripSpeed(500);
        // m_arm.setClawPosition(m_operator.arcadeWhiteLeft().getAsBoolean()?1:-1);
      }, m_arm));
    
    m_operator.arcadeWhiteLeft().onTrue(new InstantCommand(() -> {
      m_arm.setGripSpeed(1000);})

      .andThen(new InstantCommand(() -> {m_arm.unRetract();}))
      .andThen(new WaitCommand(0.6))
      
      .andThen(() -> {resetGrip();})
    );
    m_operator.arcadeWhiteRight().
    onTrue(new InstantCommand(() -> {
      m_arm.setGripSpeed(-600);},m_arm)
      .andThen(new WaitCommand(0.25))
     .andThen(new InstantCommand(() -> {m_arm.retract();}))
      .andThen(new WaitCommand(0.25))
      .andThen(() -> {m_arm.setGripSpeed(-500);})
      .andThen(new ScheduleCommand(
                      new WaitCommand(1.5)
                      .andThen(()->m_arm.unRetract()))
                      .andThen(()->resetGrip())

                      )
      // .andThen(new WaitCommand(3))
    );

  
    Translation2d floorPos = new Translation2d(-1,-0.52);
    Translation2d shutePos = new Translation2d(0.29,-0.82);
    Translation2d stationPos = new Translation2d(0.35,-0.22);

    //move to pretarget
    m_operator.arcadeBlackLeft().toggleOnTrue(
      new RunCommand(()->{
       m_arm.setArmPositionSafe(-poseEmulator(TargetSelector.getTargetIdx()).getX()+move,poseEmulator(TargetSelector.getTargetIdx()).getY());}, m_arm).until(m_operator.arcadeBlackLeft())
      .andThen(new WaitCommand(.5))
    
    );
    //move to second target and score Item
    m_operator.arcadeBlackRight().onTrue(
      new InstantCommand(()->{
        m_arm.setArmPositionSafe(-poseEmulator(TargetSelector.getTargetIdx()).getX()+move,poseEmulator(TargetSelector.getTargetIdx()).getY());}, m_arm)
        .andThen(new WaitCommand(0.8))
      .andThen(() -> {
      m_arm.setGripSpeed(-600);},m_arm)
      .andThen(new WaitCommand(0.3))
      .andThen(()->{
        m_arm.setArmPositionSafe(-poseEmulator(TargetSelector.getTargetIdx()).getX()+move,poseEmulator(TargetSelector.getTargetIdx()).getY());}, m_arm)
    .andThen(new WaitCommand(0.3))

    // .andThen( new InstantCommand(()->{ m_arm.setArmPositionSafe(-0.5,0.5);}))
    // .andThen(new WaitCommand(0.2))

    );
    m_operator.scPlus().onTrue(
      new InstantCommand( ()->{
        move+=0.05;})
     ); 
     m_operator.scMinus().onTrue(
      new InstantCommand( ()->{
         move-=0.05;})
     );

   

    m_operator.sc1().toggleOnTrue(
      new RunCommand( ()->{
         m_arm.setArmPositionSafe(-shutePos.getX(),shutePos.getY());}, m_arm)
        //  .andThen(()->move=0)
     );
    m_operator.sc2().toggleOnTrue(
      new RunCommand( ()->{
         m_arm.setArmPositionSafe(-stationPos.getX(),stationPos.getY());}, m_arm)
        //  .andThen(()->move=0)
     );
    m_operator.sc3().toggleOnTrue(
      new RunCommand( ()->{
         m_arm.setArmPositionSafe(-floorPos.getX(),floorPos.getY());}, m_arm)
        //  .andThen(()->move=0)
      
     );
    //  m_operator.scPlus().toggleOnTrue(
    //   new RunCommand( ()->{
    //      m_arm.setArmPositionSafe(-shutePos.getX(),shutePos.getY());}, m_arm)
    //  );
    //  m_operator.scMinus().toggleOnTrue(
    //   new RunCommand( ()->{
    //      m_arm.setArmPositionSafe(-floorPos.getX(),floorPos.getY());}, m_arm)
    //  );
    // m_operator.arcadeBlackRight().
    // onTrue(new InstantCommand(() -> {

    //   m_arm.setGripSpeed(-11000);})

    //   .andThen(new WaitCommand(0.5))
    //   .andThen(() -> {m_arm.setGripSpeed(0);})
    // );
    
    
    

    //X button = auto balance cmd testing
    m_driverController.x().whileTrue(new AutoBalance(m_drivetrain));
    m_driverController.y().whileTrue(new RunCommand(()->m_drivetrain.fullStop(), m_drivetrain));
    // m_driverController.b().whileTrue(new RunCommand(()->m_drivetrain.moveDistance(0.1), m_drivetrain)
    

  
    // ).onFalse(new InstantCommand(()->{m_drivetrain.accDistance(0.1);}));

    
  


    
    
    
    
    // //sets the arm to the target dual station
    // m_operator.arcadeWhiteRight().whileTrue(new InstantCommand(()->{
    // m_arm.setArmPosition(TargetMap.stationArmTargets[0]);
    // }, m_arm));

    // set the butons on the strat conm to select the target for autoalignment, NOT for autonomous
    // m_operator.sc1().onTrue(new InstantCommand(() -> {
    //   TargetSelector.setA();
    // }));
    // m_operator.sc2().onTrue(new InstantCommand(() -> {
    //   TargetSelector.setB();
    // }));
    // m_operator.sc3().onTrue(new InstantCommand(() -> {
    //   TargetSelector.setC();
    // }));


    m_operator.sc4().onTrue(new InstantCommand(() -> {
      TargetSelector.setLeft();
    }));
    m_operator.sc5().onTrue(new InstantCommand(() -> {
      TargetSelector.setCenter();
    }));
    m_operator.sc6().onTrue(new InstantCommand(() -> {
      TargetSelector.setRight();
    }));

    m_operator.scSideTop().onTrue(new InstantCommand(() -> {
      TargetSelector.setBack();
    }));
    m_operator.scSideMid().onTrue(new InstantCommand(() -> {
      TargetSelector.setMid();
    }));
    m_operator.scSideBot().onTrue(new InstantCommand(() -> {
      TargetSelector.setFront();
    }));


  }

  private void configureBindings() {

  }



  // @Log(name = "Get button Configs", tabName = "Buttons")
  public double getSliderConfig() {
    return m_operator.getLeftSlider();
  }

  // @Config.NumberSlider(name = "Set left slider")
  public void setLeftSliderConfig() {
    m_operator.getLeftSlider();
  }


  // AutonomousMap m_autonMap = new AutonomousMap(m_drivetrain, m_arm);



  public Command getAutonomousCommand2() {
    // cammad drive backwards 3 meters wait 2 seconds then drive forward 1.2 meters then back .2
    // meters using just velocity control
    System.out.println("auto");
    // return new SequentialCommandGroup(
    //     new  Command(() -> m_drivetrain.setSpeedsCONT(-1, -1)).withTimeout(2),
    //     new WaitCommand(2),
    //     new InstantCommand(() -> m_drivetrain.setSpeedsCONT(1, 1)).withTimeout(1.2),
    //     new WaitCommand(0.5),
    //     new InstantCommand(() -> m_drivetrain.setSpeedsCONT(-1, -1)).withTimeout(.2),
    //     new InstantCommand(() -> m_drivetrain.setSpeedsCONT(0, 0)).withTimeout(5));
    double drivetune = 0.94;
    double start = 1.887;
    double chargestation = 3.9;
    
   


    //  return m_arm.dropCargo2()
    // .andThen(()->m_drivetrain.smoothDrive(-0.7*drivetune,0), m_drivetrain)
    // .andThen(new WaitCommand(5.7))
    // .andThen(()->m_drivetrain.smoothDrive(0.0*drivetune,0), m_drivetrain)
    // .andThen(new WaitCommand(1).andThen(m_drivetrain.autoBalance()));
    if (m_operator.singleToggle().getAsBoolean()){
        // return new InstantCommand(()->m_drivetrain.smoothDrive(-0.7*drivetune,0), m_drivetrain)
    return m_arm.dropCargo2()
    .andThen(()->m_drivetrain.smoothDrive(-0.7*drivetune,0), m_drivetrain)
    .andThen(new WaitCommand(5.7))
    .andThen(()->m_drivetrain.smoothDrive(0.0*drivetune,0), m_drivetrain)
    .andThen(new WaitCommand(0.8))
    .andThen(new AutoBalance(m_drivetrain))
    ;
    }
    else{
     return m_arm.dropCargo2()
      .andThen(()->m_drivetrain.smoothDrive(-0.7*drivetune,0), m_drivetrain)
    .andThen(new WaitCommand(5.7))
    .andThen(()->m_drivetrain.smoothDrive(0.0*drivetune,0), m_drivetrain)
    .andThen(new WaitCommand(0.8))
    // .andThen(new AutoBalance(m_drivetrain))
    ;
    }



    
 
      
      
    
    }

  

  // @Config.Command(name = "Reset Position", tabName = "Arm PID")
  InstantCommand resetPosition = new InstantCommand(() -> {
    m_arm.resetPosition();
  }, m_arm);

  Translation2d poseEmulator(int idx){
    int col = idx%9;
    int row = (int)Math.floor(idx/9.0);
    if (col%3==1){
      // return cubeArmTargets[row];
      //0 High
      //1 mid
      //2 low
      if (row == 0){
        //high
        //angle, extent
        return new Translation2d(0.39, 0.1);
    
      }
      if (row == 1){
        //high
        //angle, extent
        return new Translation2d(0.22, -0.99);
        
      }
      if (row == 2){
        //high
        //angle, extent
        //floopos
        return new Translation2d(-1, -0.52);
        
      }
    }
    else {
      // return coneArmTargets[row];
      if (row == 0){
        //high
        //angle, extent
        return new Translation2d(0.60, 0.60);
        
      }
      if (row == 1){
        //high
        //angle, extent
        return new Translation2d(0.48, -0.31);
        
      }
      else {
        //high
        //angle, extent
        //floopos
        return new Translation2d(-1, -0.52);
        
      }
  
      }
      return new Translation2d(1, -1);
    }
     private Translation2d poseEmulator2(int idx){
        int col = idx%9;
        int row = (int)Math.floor(idx/9.0);
        if (col%3==1){
          // return cubeArmTargets[row];
          //0 High
          //1 mid
          //2 low
          if (row == 0){
            //high
            //angle, extent
            return new Translation2d(0.39, 0.1);
        
          }
          if (row == 1){
            //high
            //angle, extent
            return new Translation2d(0.22, -0.99);
            
          }
          if (row == 2){
            //high
            //angle, extent
            //floopos
            return new Translation2d(-1, -0.52);
            
          }
        }
        else {
          // return coneArmTargets[row];
          if (row == 0){
            //high
            //angle, extent
            return new Translation2d(0.44, 0.51);
            
          }
          if (row == 1){
            //high
            //angle, extent
            return new Translation2d(0.29, -0.31);
            
          }
          else{
            //high
            //angle, extent
            //floopos
            return new Translation2d(-1, -0.52);
            
          }
      
        }
        return new Translation2d(1, -1);
      }

      void resetGrip(){
        m_arm.setGripSpeed(500);
      }


// public void setDefaultarm(){
//   m_arm.setDefaultCommand(Commands.run(() -> {
//     m_arm.setArmPositionSafe(-m_operator.getRightSlider(), m_operator.getLeftSlider()) ;
    
//     // m_arm.setClawPosition(m_operator.arcadeWhiteLeft().getAsBoolean()?1:-1);
//   }, m_arm));

// }
  // public Command scoreCone(int location){

  //   if(location == 2){ //Mid
  //     return new SequentialCommandGroup(
  //       new InstantCommand(() -> m_arm.setAutoArmPos(0,0)),
  //       new WaitCommand(1),
  //       new InstantCommand(() -> m_arm.setAutoArmPos(0,0))
  //     );
  //   }

  //   if(location == 3){ //High
  //     return new SequentialCommandGroup(
  //       new InstantCommand(() -> m_arm.setAutoArmPos(0,0)),
  //       new WaitCommand(1),
  //       new InstantCommand(() -> m_arm.setAutoArmPos(0,0))
  //     );
  //   }
  // }

}
