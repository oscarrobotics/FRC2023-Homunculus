package frc.robot.subsystems.drivetrain;

<<<<<<< Updated upstream
import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.wpilibj.motorcontrol.MotorControllerGroup;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
=======

import edu.wpi.first.math.util.Units;
import edu.wpi.first.math.numbers.N2;
import edu.wpi.first.math.system.LinearSystem;
import edu.wpi.first.math.system.plant.LinearSystemId;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.geometry.Pose2d;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.DifferentialDriveKinematics;
import edu.wpi.first.math.controller.RamseteController;
import edu.wpi.first.math.estimator.DifferentialDrivePoseEstimator;

import edu.wpi.first.math.kinematics.DifferentialDriveWheelSpeeds;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.wpilibj.motorcontrol.MotorControllerGroup;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.InstantCommand;


import edu.wpi.first.wpilibj.simulation.DifferentialDrivetrainSim;

import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;


import java.util.Optional;
import java.util.function.BiConsumer;
import org.photonvision.EstimatedRobotPose;
import org.photonvision.PhotonPoseEstimator;
import org.photonvision.PhotonPoseEstimator.PoseStrategy;
>>>>>>> Stashed changes
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;
<<<<<<< Updated upstream
=======
import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.FeedbackDevice;
import com.ctre.phoenix.sensors.WPI_Pigeon2;
import com.pathplanner.lib.PathPlannerTrajectory;
import com.pathplanner.lib.commands.PPRamseteCommand;
import com.ctre.phoenix.motorcontrol.TalonFXSimCollection;
import com.ctre.phoenix.sensors.BasePigeonSimCollection;

import frc.robot.Constants;
import frc.robot.PhotonCameraWrapper;
import io.github.oblarg.oblog.Loggable;
import io.github.oblarg.oblog.annotations.Log;



public class Drivetrain extends SubsystemBase implements Loggable {
  //motors
  private final WPI_TalonFX m_leftMaster = new WPI_TalonFX(1);
  private final WPI_TalonFX m_leftDrone = new WPI_TalonFX(2);
  private final WPI_TalonFX m_rightMaster = new WPI_TalonFX(3);
  private final WPI_TalonFX m_rightDrone = new WPI_TalonFX(4);

  //gyro
  private final WPI_Pigeon2 m_gyro = new WPI_Pigeon2(0);

  //motor groups
  @Log.MotorController(name = "Left Motors", tabName = "Drivetrain")
  private final MotorControllerGroup m_leftMotors =
      new MotorControllerGroup(m_leftMaster, m_leftDrone);
  @Log.MotorController(name = "Right Motors", tabName = "Drivetrain")
  private final MotorControllerGroup m_rightMotors =
      new MotorControllerGroup(m_rightMaster, m_rightDrone);

  //camera
  public PhotonCameraWrapper pcw;

  
  //kinematics
  public final DifferentialDriveKinematics m_kinematics =
      new DifferentialDriveKinematics(Constants.wheelBaseWidth);

  // a varible used to keep track of the current commanded speed from the joystick
  // used in smooth drive as part of a low pass filter
  static double filtspeed =0;
 
 
  //odometry
  private final DifferentialDrivePoseEstimator m_poseEstimator =
      new DifferentialDrivePoseEstimator(
              m_kinematics, new Rotation2d(0), 0.0, 0.0, new Pose2d());
  //ramsete controller
  public final RamseteController m_ramseteController = new RamseteController();

 // Simulation classes help us simulate our robot
 private final BasePigeonSimCollection m_gyroSim = m_gyro.getSimCollection();
 private final TalonFXSimCollection m_leftDriveSim = m_leftMaster.getSimCollection();
 private final TalonFXSimCollection m_rightDriveSim = m_rightMaster.getSimCollection();

 final int kCountsPerRev = Constants.encoderCounts;  //Encoder counts per revolution of the motor shaft.
 final double kSensorGearRatio = Constants.gearRatio; //Gear ratio is the ratio between the *encoder* and the wheels.  
 final double kGearRatio = Constants.gearRatio; //Switch kSensorGearRatio to this gear ratio if encoder is on the motor instead of on the gearbox.
 final double kWheelRadiusInches = 3; //Wheel radius in inches
 final int k100msPerSecond = 10;


 private final Field2d m_fieldSim = new Field2d();
 private final LinearSystem<N2, N2, N2> m_drivetrainSystem =
         LinearSystemId.identifyDrivetrainSystem(1.98, 0.2, 1.5, 0.3);
 private final DifferentialDrivetrainSim m_driveSim =
         new DifferentialDrivetrainSim(
                 m_drivetrainSystem,
                 DCMotor.getFalcon500(2),
                 8,
                 Constants.wheelBaseWidth,
                 Constants.wheelRad,
                 null);
>>>>>>> Stashed changes

public class Drivetrain extends SubsystemBase {

  public Drivetrain() {
    m_rightMotors.setInverted(true);

    m_leftMaster.setNeutralMode(NeutralMode.Brake);
    m_leftDrone.setNeutralMode(NeutralMode.Brake);
    m_rightMaster.setNeutralMode(NeutralMode.Brake);
    m_rightDrone.setNeutralMode(NeutralMode.Brake);
<<<<<<< Updated upstream
=======
    // m_leftMaster.setNeutralMode(NeutralMode.Coast);
    // m_leftDrone.setNeutralMode(NeutralMode.Coast);
    // m_rightMaster.setNeutralMode(NeutralMode.Coast);
    // m_rightDrone.setNeutralMode(NeutralMode.Coast);

    //left master config
    m_leftMaster.configSelectedFeedbackSensor(TalonFXFeedbackDevice.IntegratedSensor, Constants.dkloop, Constants.kTimoutms);
    m_leftMaster.configNominalOutputForward(0);
    m_leftMaster.configNominalOutputReverse(0);
    m_leftMaster.configPeakOutputForward(1);
    m_leftMaster.configPeakOutputReverse(-1);
    m_leftMaster.configClosedloopRamp(0);
    
    m_leftMaster.config_kF(Constants.dkslot, Constants.dKf);
    m_leftMaster.config_kP(Constants.dkslot, Constants.dKp);
    m_leftMaster.config_kI(Constants.dkslot, Constants.dKi);
    m_leftMaster.config_kD(Constants.dkslot, Constants.dKd);
    m_leftMaster.config_IntegralZone(Constants.dkslot, Constants.dIz);
    

    //right master config
    m_rightMaster.configSelectedFeedbackSensor(TalonFXFeedbackDevice.IntegratedSensor, Constants.dkloop, Constants.kTimoutms);
    m_rightMaster.configNominalOutputForward(0);
    m_rightMaster.configNominalOutputReverse(0);
    m_rightMaster.configPeakOutputForward(1);
    m_rightMaster.configPeakOutputReverse(-1);
    m_rightMaster.configClosedloopRamp(0);

    m_rightMaster.config_kF(Constants.dkslot, Constants.dKf);
    m_rightMaster.config_kP(Constants.dkslot, Constants.dKp);
    m_rightMaster.config_kI(Constants.dkslot, Constants.dKi);
    m_rightMaster.config_kD(Constants.dkslot, Constants.dKd);
    m_rightMaster.config_IntegralZone(Constants.dkslot, Constants.dIz);


    //Odometry
    m_rightMotors.setInverted(true);
    
    m_leftMaster.configSelectedFeedbackSensor(FeedbackDevice.IntegratedSensor, 0, 0);
    m_rightMaster.configSelectedFeedbackSensor(FeedbackDevice.IntegratedSensor, 0, 0);
    
    m_leftMaster.setSelectedSensorPosition(0);
    m_rightMaster.setSelectedSensorPosition(0);
    m_gyro.setYaw(0);


  

   

    //Vision
    pcw = new PhotonCameraWrapper();

>>>>>>> Stashed changes
  }
// Assuming this method is part of a drivetrain subsystem that provides the necessary methods
public Command followTrajectoryCommand(PathPlannerTrajectory traj, boolean isFirstPath) {
  return new SequentialCommandGroup(
      new InstantCommand(() -> {
        // Reset odometry for the first path you run during auto
        if(isFirstPath){
            this.resetOdometry(traj.getInitialPose());
        }
      }),
      new PPRamseteCommand(
          traj, 
          this::getPose, // Pose supplier
          new RamseteController(),
          m_kinematics, // DifferentialDriveKinematics
          this::setSpeeds, // Voltage biconsumer
          true, // Should the path be automatically mirrored depending on alliance color. Optional, defaults to true
          this // Requires this drive subsystem
      )
  );
}
public void resetOdometry(Pose2d initialPose) {
  m_poseEstimator.resetPosition(m_gyro.getRotation2d(), nativeUnitsToDistanceMeters( m_leftMaster.getSelectedSensorPosition()), nativeUnitsToDistanceMeters( m_rightMaster.getSelectedSensorPosition()), initialPose);
}


  private final WPI_TalonFX m_leftMaster = new WPI_TalonFX(1);
  private final WPI_TalonFX m_leftDrone = new WPI_TalonFX(2);
  private final WPI_TalonFX m_rightMaster = new WPI_TalonFX(3);
  private final WPI_TalonFX m_rightDrone = new WPI_TalonFX(4);

  private final MotorControllerGroup m_leftMotors =
      new MotorControllerGroup(m_leftMaster, m_leftDrone);
  private final MotorControllerGroup m_rightMotors =
      new MotorControllerGroup(m_rightMaster, m_rightDrone);

<<<<<<< Updated upstream
  public final DifferentialDrive m_differentialDrive =
      new DifferentialDrive(m_leftMotors, m_rightMotors);
=======
  // private final WPI_Pigeon2 m_gyro = new WPI_Pigeon2(0); //fix canID
>>>>>>> Stashed changes

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }

  @Override
  public void simulationPeriodic() {
    // This method will be called once per scheduler run during simulation
  }

  public void arcadeDrive(double speed, double rotation) {
    m_differentialDrive.arcadeDrive(speed, rotation);
  }

  public void tankDrive(double leftSpeed, double rightSpeed) {
    m_differentialDrive.tankDrive(leftSpeed, rightSpeed);
  }

  public void stop() {
    m_differentialDrive.stopMotor();
  }

  public void setMaxOutput(double maxOutput) {
    m_differentialDrive.setMaxOutput(maxOutput);
  }
<<<<<<< Updated upstream
=======

  // BiConsumer<Double, Double> setSpeeds = (left, right) -> {
  //   m_leftMaster.set(ControlMode.Velocity, left);
  //   m_rightMaster.set(ControlMode.Velocity, right);
  //   m_leftDrone.set(ControlMode.Follower, 1);
  //   m_rightDrone.set(ControlMode.Follower, 3);
  // };
public void setSpeeds(Double left, Double right){
  m_leftMaster.set(ControlMode.Velocity, left);
    m_rightMaster.set(ControlMode.Velocity, right);
    m_leftDrone.set(ControlMode.Follower, 1);
    m_rightDrone.set(ControlMode.Follower, 3);
}
private int moveTenFeet(){  //move 10 feet in native units
  double tenFeet = Units.feetToMeters(10);
  int tenFeetNativeUnits = distanceToNativeUnits(tenFeet);
  return tenFeetNativeUnits;
}                                

public CommandBase movingCommand(){
  
  return new CommandBase() {
    @Override
    public void initialize() {}

    @Override
    public void execute() {
      m_leftMaster.set(ControlMode.Position, moveTenFeet());
      m_rightMaster.set(ControlMode.Position, moveTenFeet());
    }

    @Override
    public boolean isFinished() {
      return false;
    }
  };
>>>>>>> Stashed changes
}
