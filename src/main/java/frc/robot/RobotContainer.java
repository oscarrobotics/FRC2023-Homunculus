// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.wpilibj.shuffleboard.EventImportance;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.subsystems.drivetrain.Drivetrain;

public class RobotContainer {
  private final CommandXboxController m_driverController = new CommandXboxController(0);

  private final Drivetrain m_drivetrain = new Drivetrain();

  private final SendableChooser<Command> m_chooser = new SendableChooser<>();

  public RobotContainer() {
    Shuffleboard.getTab("Autonomous").add(m_chooser);

    configureBindings();

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
    // m_drivetrain.setDefaultCommand(Commands.run(() -> {
    //   double Smodifier = m_driverController.getRightTriggerAxis() * 0.5 + 0.5-m_driverController.getLeftTriggerAxis()*0.5;
    //   double Tmodifer =1-(( m_driverController.leftBumper().getAsBoolean()?0:1 )* (m_driverController.getRightTriggerAxis()>0.5?1:0)*0.4);
      

    //   m_drivetrain.arcadeDriveV(m_driverController.getLeftY() * Math.abs(m_driverController.getLeftY())* Smodifier * Constants.maxSpeed,
    //       m_driverController.getRightX() * Math.abs(m_driverController.getRightX())* Smodifier * Tmodifer*Constants.maxTurn);
    // }, m_drivetrain));
  //  
    // m_drivetrain.setDefaultCommand(Commands.run(() -> {
    //   m_drivetrain.arcadeDriveV(m_driverController.getLeftY(), m_driverController.getRightX());
    //   double modifier = m_driverController.getRightTriggerAxis() * 0.5 + 0.5;
    //   m_drivetrain.arcadeDriveV(m_driverController.getLeftY() * modifier,
    //       m_driverController.getRightX() * modifier);
    // }, m_drivetrain));
    m_drivetrain.setDefaultCommand(Commands.run(() -> {
        double Smodifier = m_driverController.getRightTriggerAxis() * 0.5 + 0.5-m_driverController.getLeftTriggerAxis()*0.5;
        double Tmodifer =1-(( m_driverController.leftBumper().getAsBoolean()?0:1 )* (m_driverController.getRightTriggerAxis()>0.5 && m_driverController.getLeftTriggerAxis()<0.5?1:0)*0.4);
        
  
        m_drivetrain.smoothDrive(m_driverController.getLeftY() * Math.abs(m_driverController.getLeftY())* Smodifier * Constants.maxSpeed,
            m_driverController.getRightX() * Math.abs(m_driverController.getRightX())* Smodifier * Tmodifer*Constants.maxTurn);
      }, m_drivetrain));

  }

  private void configureBindings() {}

  public Command getAutonomousCommand() {
    return m_chooser.getSelected();
  }
}
