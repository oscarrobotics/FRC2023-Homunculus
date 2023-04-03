package frc.robot;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.RobotState;



public class TargetMap {

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

      coneArmTargets[2] = new Translation2d(0.25, 0.35); //0
      coneArmTargets[1] = new Translation2d(0.53, 1.70); //87 4inchs higher
      coneArmTargets[0] = new Translation2d(1.01, 2.10); //117 4inchs higher

      cubeArmTargets[2] = new Translation2d(0.25, 0.35); //0
      cubeArmTargets[1] = new Translation2d(0.53, 0.95); //60 6inchs higher
      cubeArmTargets[0] = new Translation2d(1.01, 1.25); //90 6inchs higher

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

  public static Pose2d getBlueTargetPose(int target) {
    return blueTargetPoses[target%9];
  }

  public static Pose2d getRedTargetPose(int target) {
    return redTargetPoses[target%9];
  }
  public static Pose2d getBlueStationPose(int target) {
    return blueStationPose[target%4];
  }
  public static Pose2d getRedStationPose(int target) {
    return redStationPose[target%4];
  }
  public static Pose2d getBlueChargeingPose(int target) {
    return blueChargeingPose[target%3];
  }
  public static Pose2d getRedChargeingPose(int target) {
    return redChargeingPose[target%3];
  }
  public static Pose2d getBlueChargeingPose2bot(int target) {
    return blueChargeingPose2bot[target%2];
  }
  public static Pose2d getRedChargeingPose2bot(int target) {
    return redChargeingPose2bot[target%2];
  }
  public static Translation2d getRedConeArmTarget(int target) {
    return coneArmTargets[target%3];
  }
  public static Translation2d getBlueConeArmTarget(int target) {
    return coneArmTargets[target%3];
  }
  public static Translation2d getRedCubeArmTarget(int target) {
    return cubeArmTargets[target%3];
  }
  public static Translation2d getBlueCubeArmTarget(int target) {
    return cubeArmTargets[target%3];
  }
  public static Translation2d getRedStationArmTarget(int target) {
    return stationArmTargets[target%4];
  }
  public static Translation2d getBlueStationArmTarget(int target) {
    return stationArmTargets[target%4];

  }
public static Pose2d getTargetPose(int idx){
  //get alliance color
   if (DriverStation.getAlliance() == DriverStation.Alliance.Red) {
    return redTargetPoses[idx];
  } else {
    return blueTargetPoses[idx];
  }
}
public static Pose2d getStationPose(int idx){
  //get alliance color
   if (DriverStation.getAlliance() == DriverStation.Alliance.Red) {
    return redStationPose[idx];
  } else {
    return blueStationPose[idx];
  }
  
}
public static Pose2d getChargeingPose(int idx){
  //get alliance color
   if (DriverStation.getAlliance() == DriverStation.Alliance.Red) {
    return redChargeingPose[idx];
  } else {
    return blueChargeingPose[idx];
  }
}
public static Pose2d getChargeingPose2bot(int idx){
  //get alliance color
   if (DriverStation.getAlliance() == DriverStation.Alliance.Red) {
    return redChargeingPose2bot[idx];
  } else {
    return blueChargeingPose2bot[idx];
  }
}
public Translation2d getArmTarget(int idx){
  //get alliance color
  int col = idx%9;
  int row = (int)Math.floor(idx/9.0);
  if (col%3==1){
    return cubeArmTargets[row];
  }
  else 
    return coneArmTargets[row];


}
  
}
