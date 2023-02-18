package frc.robot.subsystems.drivetrain;

import edu.wpi.first.apriltag.AprilTag;
import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.kinematics.DifferentialDriveKinematics;
import edu.wpi.first.math.kinematics.DifferentialDriveOdometry;
import edu.wpi.first.math.kinematics.DifferentialDriveWheelSpeeds;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.wpilibj.motorcontrol.MotorControllerGroup;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import java.io.IOException;
import java.util.ArrayList;
import java.util.List;
import org.photonvision.PhotonCamera;
import org.photonvision.PhotonPoseEstimator;
import org.photonvision.PhotonPoseEstimator.PoseStrategy;
import org.photonvision.targeting.PhotonTrackedTarget;
import com.ctre.phoenix.motorcontrol.FeedbackDevice;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.TalonFXFeedbackDevice;
import com.ctre.phoenix.motorcontrol.can.TalonFXConfiguration;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;
import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.sensors.Pigeon2;
import com.ctre.phoenix.sensors.WPI_Pigeon2;
import frc.robot.Constants;

public class Drivetrain extends SubsystemBase {
  
  private final WPI_TalonFX m_leftMaster = new WPI_TalonFX(1);
  private final WPI_TalonFX m_leftDrone = new WPI_TalonFX(2);
  private final WPI_TalonFX m_rightMaster = new WPI_TalonFX(3);
  private final WPI_TalonFX m_rightDrone = new WPI_TalonFX(4);

  private final DifferentialDriveOdometry m_odometry;

  private final Pigeon2 m_gyro = new Pigeon2(0);

  private final MotorControllerGroup m_leftMotors =
      new MotorControllerGroup(m_leftMaster, m_leftDrone);
  private final MotorControllerGroup m_rightMotors =
      new MotorControllerGroup(m_rightMaster, m_rightDrone);


  // public final DifferentialDrive m_differentialDrive =
  //     new DifferentialDrive(m_leftMotors, m_rightMotors);

  public final DifferentialDriveKinematics m_kinematics =
      new DifferentialDriveKinematics(Constants.wheelbwidth);

  public Drivetrain() {
    m_rightMotors.setInverted(true);
    m_rightMaster.setInverted(true);
    m_rightDrone.setInverted(true);

    m_leftMaster.setNeutralMode(NeutralMode.Brake);
    m_leftDrone.setNeutralMode(NeutralMode.Brake);
    m_rightMaster.setNeutralMode(NeutralMode.Brake);
    m_rightDrone.setNeutralMode(NeutralMode.Brake);
    // m_leftMaster.setNeutralMode(NeutralMode.Coast);
    // m_leftDrone.setNeutralMode(NeutralMode.Coast);
    // m_rightMaster.setNeutralMode(NeutralMode.Coast);
    // m_rightDrone.setNeutralMode(NeutralMode.Coast);

      
    m_leftMaster.configSelectedFeedbackSensor(TalonFXFeedbackDevice.IntegratedSensor, Constants.dkloop, Constants.kTimoutms);
    m_leftMaster.configNominalOutputForward(0);
    m_leftMaster.configNominalOutputReverse(0);
    m_leftMaster.configPeakOutputForward(1);
    m_leftMaster.configPeakOutputReverse(-1);
    m_leftMaster.configClosedloopRamp(.5);
    
    m_leftMaster.config_kF(Constants.dkslot, Constants.dKf);
    m_leftMaster.config_kP(Constants.dkslot, Constants.dKp);
    m_leftMaster.config_kI(Constants.dkslot, Constants.dKi);
    m_leftMaster.config_kD(Constants.dkslot, Constants.dKd);
    m_leftMaster.config_IntegralZone(Constants.dkslot, Constants.dIz);
    


    m_rightMaster.configSelectedFeedbackSensor(TalonFXFeedbackDevice.IntegratedSensor, Constants.dkloop, Constants.kTimoutms);
    m_rightMaster.configNominalOutputForward(0);
    m_rightMaster.configNominalOutputReverse(0);
    m_rightMaster.configPeakOutputForward(1);
    m_rightMaster.configPeakOutputReverse(-1);
    m_rightMaster.configClosedloopRamp(.5);

    m_rightMaster.config_kF(Constants.dkslot, Constants.dKf);
    m_rightMaster.config_kP(Constants.dkslot, Constants.dKp);
    m_rightMaster.config_kI(Constants.dkslot, Constants.dKi);
    m_rightMaster.config_kD(Constants.dkslot, Constants.dKd);
    m_rightMaster.config_IntegralZone(Constants.dkslot, Constants.dIz);




    //Odometry
    
    
    m_leftMaster.configSelectedFeedbackSensor(FeedbackDevice.IntegratedSensor, 0, 0);
    m_rightMaster.configSelectedFeedbackSensor(FeedbackDevice.IntegratedSensor, 0, 0);
    
    m_leftMaster.setSelectedSensorPosition(0);
    m_rightMaster.setSelectedSensorPosition(0);

    m_odometry = new DifferentialDriveOdometry(new Rotation2d(0),0,0);
        new Rotation2d(m_gyro.getYaw());

    m_leftMaster.getSelectedSensorPosition();
    m_rightMaster.getSelectedSensorPosition();  

    //Vision
      PhotonCamera camera = new PhotonCamera("photonvision");
      Transform3d robotToCam = new Transform3d(new Translation3d(0.5, 0, 0.5), new Rotation3d(0, 0, 0));
      List<AprilTag> apriltagss = new ArrayList<AprilTag>();
      apriltagss.add(new AprilTag(0, new Pose3d(new Pose2d(new Translation2d(0,0), new Rotation2d(0)))));
      apriltagss.add(new AprilTag(0, new Pose3d(new Pose2d(new Translation2d(0,0), new Rotation2d(0)))));
      AprilTagFieldLayout layout = new AprilTagFieldLayout(apriltagss, 0, 0);
      try {
        layout = AprilTagFields.k2023ChargedUp.loadAprilTagLayoutField();
      }
      catch(IOException e){
        printf("file does not exist");
        
      }

      PhotonPoseEstimator photonPoseEstimator = new PhotonPoseEstimator(layout, PoseStrategy.CLOSEST_TO_REFERENCE_POSE, camera, robotToCam);
    
    //TODO: get targets, get results, get data from targets, etc.
      // var result = camera.getLatestResult();
      // boolean hasTargets = result.hasTargets(); 

      // PhotonTrackedTarget target = result.getBestTarget();
      // List<PhotonTrackedTarget> targets = result.getTargets();
      // // double poseAmbiguity = target.getPoseAmbiguity();
      // Transform3d bestCameraToTarget = target.getBestCameraToTarget();
      // Transform3d alternateCameraToTarget = target.getAlternateCameraToTarget();
  }

  // private final WPI_Pigeon2 m_gyro = new WPI_Pigeon2(0); //fix canID

  private void printf(String string) {}

  @Override
  public void periodic() {
    // This method will be called once per scheduler run

    // System.out.println(String.format("Roll: %f, Pitch: %f, Yaw: %f", m_gyro.getRoll(), m_gyro.getPitch(), m_gyro.getYaw()));
  }

  @Override
  public void simulationPeriodic() {
    // This method will be called once per scheduler run during simulation
  }

  // public void arcadeDrive(double speed, double rotation) {
  //   m_differentialDrive.arcadeDrive(speed, rotation);
  // }

  public void arcadeDriveV(double speed, double rotation){
    //use the differntial drive invers kinematics class to set the motor controllers directly in velocity control mode
    
    DifferentialDriveWheelSpeeds wheelSpeeds = m_kinematics.toWheelSpeeds(new ChassisSpeeds(speed, 0, rotation));


    // 2048 units per rev
    // 600 : 100ms per min

    double metertorpm = Constants.wheelrad*2*Math.PI ; //convertion from meters per second to rev per second
    double rpmtounit = 2048/10; //convertion from rpm to units per 100ms

    double leftspeedtalon = wheelSpeeds.leftMetersPerSecond / metertorpm * rpmtounit; //unit/100ms
    double rightspeedtalon = wheelSpeeds.rightMetersPerSecond / metertorpm * rpmtounit; //unit/100ms
    System.out.print(leftspeedtalon);
    System.out.print("  ");
    System.out.println(rightspeedtalon);

    m_leftMaster.set(ControlMode.Velocity, leftspeedtalon);
    m_rightMaster.set(ControlMode.Velocity, rightspeedtalon);
    m_leftDrone.set(ControlMode.Follower, 1);
    m_rightDrone.set(ControlMode.Follower, 3);
    


   }

  // public void curvatureDrive(double speed, double rotation, boolean isQuickTurn) {
  //   m_differentialDrive.curvatureDrive(speed, rotation, isQuickTurn);
  // }

  // public void tankDrive(double leftSpeed, double rightSpeed) {
  //   m_differentialDrive.tankDrive(leftSpeed, rightSpeed);
  // }



  public void stop() {
    // m_differentialDrive.stopMotor();
    m_leftMotors.stopMotor();
    m_rightMotors.stopMotor();
  }

  public void setMaxOutput(double maxOutput) {
    // m_differentialDrive.setMaxOutput(maxOutput);
  
  }
}
