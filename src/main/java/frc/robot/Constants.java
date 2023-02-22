package frc.robot;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.util.Units;


public final  class Constants {

  public static final int kTimoutms = 30;

  public static final double wheelrad = Units.inchesToMeters(3);
  public static final double wheelbwidth = Units.inchesToMeters(28); // test robot
  // public final double wheelbwidth = Units.inchesToMeters(26); // new robot
  public static final double wheelblength =30;
  public static final double gearratio = 10; // test robot
  // public final double gearratio = 9.8; // new robot
  public static final double maxSpeed = 4.0; //m/s

  public static final double dKf = 1023.0 / 2066.0;
  public static final double dKp = 0.1;
  public static final double dKi = 0.001;
  public static final double dKd = 5;
  public static final double dIz= 300;
  public static final int encodercounts = 2048;

  
  public static final int dkslot = 0;
  public static final int dkloop = 0;

  static class FieldConstants {
    static final double length = Units.feetToMeters(54);
    static final double width = Units.feetToMeters(27);
}

  static class VisionConstants {
    static final Transform3d robotToCam =
            new Transform3d(
                    new Translation3d(0.5, 0.0, 0.5),
                    new Rotation3d(
                            0, 0,
                            0)); // Cam mounted facing forward, half a meter forward of center, half a meter up
    // from center.
    static final String cameraName = "photonvision";
}
  
}
