// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.wpilibj.shuffleboard.EventImportance;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
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

    m_drivetrain.setDefaultCommand(Commands.run(() -> {
      double modifier = m_driverController.getRightTriggerAxis() * 0.5 + 0.5;
      m_drivetrain.arcadeDrive(m_driverController.getLeftY() * modifier,
          m_driverController.getRightX() * modifier);
    }, m_drivetrain));
  }

  private void configureBindings() {}

  public Command getAutonomousCommand() {
    return m_chooser.getSelected();
  }
}
