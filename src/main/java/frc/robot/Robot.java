// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;


public class Robot extends TimedRobot {
  private Command m_autonomousCommand;

  private RobotContainer m_robotContainer;

  
  
  




  final double CAMERA_HEIGHT_METERS = Units.inchesToMeters(24);
  final double CAMERA_ANGLE_RADIANS = Units.degreesToRadians(0);
  
  //Tune PID const via sysID
  final double LINEAR_P = 0.1;
  final double LINEAR_D = 0.0;
  PIDController forwardController = new PIDController(LINEAR_P, 0, LINEAR_D);

  final double ANGULAR_P = 0.1;
  final double ANGULAR_D = 0.0;
  PIDController turnController = new PIDController(ANGULAR_P, 0, ANGULAR_D);

  XboxController xboxController;

  @Override
  public void robotInit() {
    m_robotContainer = new RobotContainer();
  }

  @Override
  public void robotPeriodic() {
    CommandScheduler.getInstance().run();
  }

  @Override
  public void disabledInit() {}

  @Override
  public void disabledPeriodic() {}

  @Override
  public void disabledExit() {}

  @Override
  public void autonomousInit() {
    m_autonomousCommand = m_robotContainer.getAutonomousCommand();

    if (m_autonomousCommand != null) {
      m_autonomousCommand.schedule();
    }
  }

  @Override
  public void autonomousPeriodic() {}

  @Override
  public void autonomousExit() {}

  @Override
  public void teleopInit() {
    if (m_autonomousCommand != null) {
      m_autonomousCommand.cancel();
    }
  }

  @Override
  public void teleopPeriodic() {
    // double forwardSpeed;
    // double rotationSpeed;

    // forwardSpeed = -xboxController.getRightY();

    // if(xboxController.getAButton()){
    //   var result = camera.getLatestResult();
      
    //   if(result.hasTargets()){
    //     rotationSpeed = -turnController.calculate(result.getBestTarget().getYaw(), 0);
    //     // double range = PhotonUtils.calculateDistanceToTargetMeters(rotationSpeed, rotationSpeed, forwardSpeed, rotationSpeed);
    //   } else { 
    //     rotationSpeed = 0; 
    //   }
    // } else {
    //   rotationSpeed = xboxController.getLeftX();
    // }

    // m_Drivetrain.arcadeDrive(forwardSpeed, rotationSpeed);
  }

  @Override
  public void teleopExit() {}

  @Override
  public void testInit() {
    CommandScheduler.getInstance().cancelAll();
  }

  @Override
  public void testPeriodic() {}

  @Override
  public void testExit() {}
}
