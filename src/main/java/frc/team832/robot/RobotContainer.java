package frc.team832.robot;

import org.photonvision.PhotonCamera;

import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj.Compressor;
import edu.wpi.first.wpilibj.PneumaticsModuleType;
import edu.wpi.first.wpilibj.livewindow.LiveWindow;
import frc.team832.robot.subsystems.DrivetrainSubsystem;
import frc.team832.robot.subsystems.VisionSubsystem;
import lombok.Getter;
import lombok.Setter;



/**
 * This class is where the bulk of the robot should be declared. Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of the robot (including
 * subsystems, commands, and button mappings) should be declared here.
 */

public class RobotContainer {
  /** Control system objects **/
  @Setter
  @Getter
  private Pose2d autoPose;

  /** Subsystems **/
  public final DrivetrainSubsystem drivetrain = new DrivetrainSubsystem();

  public final VisionSubsystem visionSubsystem = new VisionSubsystem();
  /** HID Controllers **/

  /** Autonomous Selector **/
  public final AutonomousSelector autoSelector = new AutonomousSelector();
  private final SlewRateLimiter driveRateLimiter = new SlewRateLimiter(6);
  private final SlewRateLimiter turnRateLimiter = new SlewRateLimiter(3);

  /** The container for the robot. Contains subsystems, OI devices, and commands. */
  public RobotContainer() {
    PhotonCamera.setVersionCheckEnabled(false);
    LiveWindow.disableAllTelemetry();


    /* Arcade Drive Commands */

  }
}
