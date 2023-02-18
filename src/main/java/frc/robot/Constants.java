package frc.robot;
import edu.wpi.first.math.util.Units;

public final  class Constants {

  public static final int kTimoutms = 30;

  public static final double wheelrad = Units.inchesToMeters(3);
  public static final double wheelbwidth = Units.inchesToMeters(28); // test robot
  // public final double wheelbwidth = Units.inchesToMeters(26); // new robot
  public static final double wheelblength =30;
  
  public static final double maxSpeed = 8.0; //m/s

  public static final double dKf = 1023.0 / 2066.0;
  public static final double dKp = 0.1;
  public static final double dKi = 0.001;
  public static final double dKd = 5;
  public static final double dIz= 300;

  
  public static final int dkslot = 0;
  public static final int dkloop = 0;
  
}
