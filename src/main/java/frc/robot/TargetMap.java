package frc.robot;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;

public  class TargetMap {

  public static Pose2d[] blueTargetPoses = new Pose2d[9];
  public static Pose2d[] redTargetPoses = new Pose2d[9];
  public static Translation2d[] coneArmTargets= new Translation2d[3];
  public static Translation2d[] cubeArmTargets= new Translation2d[3];

  public static Pose2d[] blueStationPose = new Pose2d[4];
  public static Pose2d[] redStationPose = new Pose2d[4];
  public static Translation2d[] stationArmTargets = new Translation2d[4];

  public static Pose2d[] blueChargeingPose = new Pose2d[3];
  public static Pose2d[] redChargeingPose = new Pose2d[3];

  public static Pose2d[] blueChargeingPose2bot = new Pose2d[2];
  public static Pose2d[] redChargeingPose2bot = new Pose2d[2];

  TargetMap() {
      //collum 1 is bottom of field
      blueTargetPoses[0] = new Pose2d(new Translation2d(2, 0.52), new Rotation2d(Math.toRadians(180)));
      blueTargetPoses[1] = new Pose2d(new Translation2d(2, 1.06), new Rotation2d(Math.toRadians(180)));
      blueTargetPoses[2] = new Pose2d(new Translation2d(2, 1.62), new Rotation2d(Math.toRadians(180)));
      blueTargetPoses[3] = new Pose2d(new Translation2d(2, 2.19), new Rotation2d(Math.toRadians(180)));
      blueTargetPoses[4] = new Pose2d(new Translation2d(2, 2.75), new Rotation2d(Math.toRadians(180)));
      blueTargetPoses[5] = new Pose2d(new Translation2d(2, 3.31), new Rotation2d(Math.toRadians(180)));
      blueTargetPoses[6] = new Pose2d(new Translation2d(2, 3.87), new Rotation2d(Math.toRadians(180)));
      blueTargetPoses[7] = new Pose2d(new Translation2d(2, 4.43), new Rotation2d(Math.toRadians(180)));  
      blueTargetPoses[8] = new Pose2d(new Translation2d(2, 4.98), new Rotation2d(Math.toRadians(180)));

      redTargetPoses[8] = new Pose2d(new Translation2d(14.54, 0.52), new Rotation2d(Math.toRadians(0)));
      redTargetPoses[7] = new Pose2d(new Translation2d(14.54, 1.06), new Rotation2d(Math.toRadians(0)));
      redTargetPoses[6] = new Pose2d(new Translation2d(14.54, 1.62), new Rotation2d(Math.toRadians(0)));
      redTargetPoses[5] = new Pose2d(new Translation2d(14.54, 2.19), new Rotation2d(Math.toRadians(0)));
      redTargetPoses[4] = new Pose2d(new Translation2d(14.54, 2.75), new Rotation2d(Math.toRadians(0)));
      redTargetPoses[3] = new Pose2d(new Translation2d(14.54, 3.31), new Rotation2d(Math.toRadians(0)));
      redTargetPoses[2] = new Pose2d(new Translation2d(14.54, 3.87), new Rotation2d(Math.toRadians(0)));
      redTargetPoses[1] = new Pose2d(new Translation2d(14.54, 4.43), new Rotation2d(Math.toRadians(0)));
      redTargetPoses[0] = new Pose2d(new Translation2d(14.54, 4.98), new Rotation2d(Math.toRadians(0)));

      coneArmTargets[0] = new Translation2d(0.25, 0.25); //0
      coneArmTargets[1] = new Translation2d(0.61, 97); //87 4inchs higher
      coneArmTargets[2] = new Translation2d(1.01, 0.127); //117 4inchs higher

      cubeArmTargets[0] = new Translation2d(0.25, 0.25); //0
      cubeArmTargets[1] = new Translation2d(0.61, 75); //60 6inchs higher
      cubeArmTargets[2] = new Translation2d(1.01, 0.105); //90 6inchs higher

      //charging stations
      //top to bottom third is ground sigle sitation 4th is lined up with single station

      redStationPose[0] = new Pose2d(new Translation2d(1.50, 7.29), new Rotation2d(Math.toRadians(171)));
      redStationPose[1] = new Pose2d(new Translation2d(1.50, 6.25), new Rotation2d(Math.toRadians(191)));
      redStationPose[2] = new Pose2d(new Translation2d(3.6, 7.2), new Rotation2d(Math.toRadians(165)));
      redStationPose[3] = new Pose2d(new Translation2d(2.70, 6.90), new Rotation2d(Math.toRadians(98)));

      blueStationPose[0] = new Pose2d(new Translation2d(15, 7.29), new Rotation2d(Math.toRadians(9.0)));
      blueStationPose[1] = new Pose2d(new Translation2d(15, 6.25), new Rotation2d(Math.toRadians(345)));
      blueStationPose[2] = new Pose2d(new Translation2d(12.9, 7.2), new Rotation2d(Math.toRadians(15)));
      blueStationPose[3] = new Pose2d(new Translation2d(13.8, 6.90), new Rotation2d(Math.toRadians(82)));

      stationArmTargets[0] = new Translation2d(0.90, 1.05); //95
      stationArmTargets[1] = new Translation2d(0.90, 1.05); //95
      stationArmTargets[2] = new Translation2d(0.65, .2); //ground
      stationArmTargets[3] = new Translation2d(0.51, .69); //.69


      //charging stations
      // goes from bottom to top facing the grid
      blueChargeingPose[0] = new Pose2d(new Translation2d(3.90, 1.93), new Rotation2d(Math.toRadians(180)));
      blueChargeingPose[1] = new Pose2d(new Translation2d(3.90, 2.72), new Rotation2d(Math.toRadians(180)));
      blueChargeingPose[2] = new Pose2d(new Translation2d(3.90, 3.52), new Rotation2d(Math.toRadians(180)));

      redChargeingPose[0] = new Pose2d(new Translation2d(12.64, 1.93), new Rotation2d(Math.toRadians(0)));
      redChargeingPose[1] = new Pose2d(new Translation2d(12.64, 2.72), new Rotation2d(Math.toRadians(0)));
      redChargeingPose[2] = new Pose2d(new Translation2d(12.64, 3.52), new Rotation2d(Math.toRadians(0)));

      // charging staion posers that onlly leave room for 2 robots
      blueChargeingPose2bot[0] = new Pose2d(new Translation2d(3.90, 2.2), new Rotation2d(Math.toRadians(180)));
      blueChargeingPose2bot[1] = new Pose2d(new Translation2d(3.90, 3.3), new Rotation2d(Math.toRadians(180)));

      redChargeingPose2bot[0] = new Pose2d(new Translation2d(12.64, 2.2), new Rotation2d(Math.toRadians(0)));
      redChargeingPose2bot[1] = new Pose2d(new Translation2d(12.64, 3.3), new Rotation2d(Math.toRadians(0)));


  }
  
}
