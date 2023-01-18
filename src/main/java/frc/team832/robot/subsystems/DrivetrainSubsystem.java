package frc.team832.robot.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

import static frc.team832.robot.Constants.DriveTrainConstants.*;

import com.ctre.phoenix.motorcontrol.can.TalonFX;
import com.ctre.phoenix.sensors.WPI_Pigeon2;

public class DrivetrainSubsystem extends SubsystemBase {

  /*
   * I'm considering just pulling out the talon directly, but it'll be encoder methods copy and
   * paste central. I could create A new class but is the abstraction worth it I wonder
   */
  public final TalonFX leftMasterMotor = new TalonFX(LEFT_MASTER_TALON_ID);
  public final TalonFX leftSlaveMotor = new TalonFX(LEFT_SLAVE_TALON_ID);
  public final TalonFX rightMasterMotor = new TalonFX(RIGHT_MASTER_TALON_ID);
  public final TalonFX rightSlaveMotor = new TalonFX(RIGHT_SLAVE_TALON_ID);
  public final WPI_Pigeon2 imu = new WPI_Pigeon2(PIGEON_ID);
}
