package frc.robot.subsystems.arm;
import frc.robot.subsystems.arm.Extend;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.WaitCommand;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.TargetMap;
import frc.robot.TargetSelector;
import io.github.oblarg.oblog.Loggable;
import io.github.oblarg.oblog.annotations.Config;
import io.github.oblarg.oblog.annotations.Log;
import java.lang.annotation.Target;
import com.revrobotics.AbsoluteEncoder;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.CANSparkMax.ControlType;
import com.revrobotics.CANSparkMax.IdleMode;
import com.revrobotics.CANSparkMax.SoftLimitDirection;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import com.revrobotics.SparkMaxAbsoluteEncoder.Type;
import com.revrobotics.SparkMaxPIDController;

public class Arm extends SubsystemBase implements Loggable{
  
public final Extend s_extend;
public final Raise s_raise;

public final CANSparkMax m_gripMotor;



RelativeEncoder m_gripEncoder;




SparkMaxPIDController m_gripPID;


private final double k_maxLengthPos = 55;//38.3//postion at the max length we mesures in units
private final double k_minLengthPos = 0;//postion at the max length we mesures in units



private final double k_maxLeanglePos = 116 ; //max length of the angle control in units
private final double k_minLeanglePos = 0 ; //max length of the angle control in units
private final double k_maxClawPos = 43.3; //43.3
private final double k_minClawPos = 0;

// private final double k_rangeLengthPos = k_maxLengthPos - k_minLengthPos;
// private final double k_rangeLeanglePos = k_maxLeanglePos- k_minLeanglePos;
final static double k_rangeExtentPos = 55;
final static double k_rangeLeanglePos =116;//needs to be tuned, adjust tilll the arm is at the right angle at bottom should be nearly correct

private final double k_rangeClawPos = k_maxClawPos - k_minClawPos; //also should be checked and tuned


private final double k_maxHeight = Units.inchesToMeters((12*6.25)) ;// height in feet that the robot is alowed to be

private final double k_pivotHeight = Units.inchesToMeters(21); // height in inches that the pivot point oi the arm is of the floor
private double k_minArmHeight = Units.inchesToMeters(5); // height in inches that the arm is alowed to be at the bottom
private final double d_minArmHeight = 5;


private final double k_defaultLength = 0; // the length we want the ar m to be most of the time 
private final double k_defaultLeanglle= 0;//the lenght of the angle  motor we want most of the time 

private final double k_coneClaw=1;
private final double k_cubeClaw=0.1;




// private final double k_pivotLenght = Units.inchesToMeters(17.5);//? the hieght of the arm pivot from the base of the angle actuator fill in actual
private final double k_pivotLenght = Units.inchesToMeters(16.5);
private final double k_pivotOffset = Units.inchesToMeters(5.25); //distance between the arm pirvot point and the place where the angle motor attaches to the arm, 
// private final double k_minLeangle= Units.inchesToMeters(13.4);//used for kinematics
private final double k_minLeangle= Units.inchesToMeters(14.87);
// private final double k_maxLeangle= Units.inchesToMeters(21.2);//used for kinemati
private final double k_maxLeangle= Units.inchesToMeters(21.0);
private final double k_minLength= Units.inchesToMeters(30.75);
private final double k_maxLength= Units.inchesToMeters(70);//59
private final double k_columToFront = Units.inchesToMeters(18); //distance of the colloum from the front of the robot

final static double k_ticksPerInchExtend=1;
private final double k_ticksPerInchRaise=1;
private final double k_ticksPerInchGrip= 1;

  //pid constants

//grip
  public final double kPG = 1;
  public final double kIG = 0;
  public final double kDG = 0;
  public final double kIzG = 0;
  public final double kFFG = 0; 
  public final double kMaxOutputG = 0.3;//grip open?
  public final double kMinOutputG = 0.3;//grip close?
  public final double maxRPMG = 5;
  public final double maxAccelG = 2;

  
  public double vExtendSetPos = 0;

  public double vRaisePos = 0;
  public double vGripPos = 0;

  // @Log.Graph(name = "minArmHeight")
  public double vMaxExtention = 0;
  // @Log.Graph(name = "minAgnle")
  public double vMinAngle = 0;
  public double k_targetRaisePosHigh;
  public double k_targetExtPosHigh;
  public double k_targetGripPosHigh;


// private final AbsoluteEncoder m_Encoder;

@Log(name = "Grip Motor Temp", tabName = "Extend", rowIndex =4 , columnIndex = 5)
     public double gripMotorTemp(){
      return m_gripMotor.getMotorTemperature();
     }

//Sets max velocity, acceleration, and state
 
  public Arm(){
    s_extend = new Extend();
    s_raise = new Raise();

   m_gripMotor = new CANSparkMax(22,MotorType.kBrushless) ;
   m_gripMotor.setInverted(true);
   //cuewnt limit
   
   m_gripMotor.setSmartCurrentLimit(10,28,0);

   // neutral mode
      
   m_gripMotor.setIdleMode(IdleMode.kBrake);

  //encoder
    
    m_gripEncoder = m_gripMotor.getEncoder();

    // pid controllers
   
    m_gripPID = m_gripMotor.getPIDController();
    
    m_gripPID.setP(kPG,0);
    m_gripPID.setI(kIG,0);
    m_gripPID.setD(kDG,0);
    m_gripPID.setIZone(kIzG,0);
    m_gripPID.setFF(kFFG,0);
    m_gripPID.setOutputRange(-kMinOutputG, kMaxOutputG,0);

    int kSlotIdxG = 0;
    m_gripPID.setSmartMotionMaxVelocity(maxRPMG, kSlotIdxG);
    m_gripPID.setSmartMotionMinOutputVelocity(-maxRPMG, kSlotIdxG);
    m_gripPID.setSmartMotionMaxAccel(maxAccelG, kSlotIdxG);
    m_gripPID.setSmartMotionAllowedClosedLoopError(10, kSlotIdxG);

  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    // checkIfStalled();
    s_extend.isSafeTemp();
    s_raise.isSafeTemp();
    gripMotorTemp();
  }







 public void setArmPosition(Translation2d position){
    //takes in a pose and sets the arm to that position
    Translation2d setpoints = cartToPosRatio(position.getX(), position.getY());
    System.out.println("position"+position);
    System.out.println("setpoints"+ setpoints);
    
    setRaisePosition(setpoints.getX()); 
    setExtendPositionArbFF(setpoints.getY());

 }
 

  // public void setArmPositionSafe (double angleR, double extentR){
  //     //takes in Setpoints and sets the arm to that position 
  //     //;retracts the arm if the arm if the angle would makie it incorectly positioned
  //     //extent =  extentPosToExtent(extentPos)
  //     //angle = anglePosToAngle(anglePos)
  //    //height = extent * sin(angle) + pivotHeight
  //     //length = extent * cos(angle) -  columtToFront
  //     //raisePos = angleToLeangle(angle)
  //     //maxHeight = 1.87m
  //     //minHeight = 0.15m
  //     //maxLength = 1.1m
  //     //pivotHeight = 0.33m
  //     //columtToFront = 0.46m
  //     //leangle is the angle of the arm
  //     //raise is the angle of the arm
  //     double leanglePos = s_raise.mapInput(angleR);
  //     double extentPos = s_extend.mapInput(extentR);
      
  //     double angle = leanglePosToAngle(leanglePos);
  //     double extent = extentPosToExtent(extentPos);

  //     System.out.println("angle"+angle);
  //     double height = extent * Math.sin(Math.toRadians( angle)) + k_pivotHeight;
  //     double length = extent * Math.cos(Math.toRadians( angle)) - k_columToFront;
  //     System.out.println("height"+ height);
  //     System.out.println("length"+length);

  //     if (height > k_maxHeight){
  //       extentPos = extentToExtentPos ( (k_maxHeight- k_pivotHeight ) / Math.sin(Math.toRadians( angle)));
  //       extent = extentPosToExtent(extentPos);
  //       length = extent * Math.cos(Math.toRadians( angle)) - k_columToFront;
  //       // angle = Math.atan((k_maxHeight-k_pivotHeight)/length)
  //       ;
  //       // leanglePos = angleToLeanglePos(angle);
  //     }
  //     if(height < k_minArmHeight){
  //       extentPos = extentToExtentPos((( k_pivotHeight-k_minArmHeight )) / Math.sin(angle)*-1);
  //       extent = extentPosToExtent(extentPos);
  //       length = extent * Math.cos(Math.toRadians( angle)) - k_columToFront;
  //       // angle = Math.toDegrees(Math.atan((k_pivotHeight-k_minArmHeight)/length))*-1;
  //       // leanglePos = angleToLeanglePos(angle);
  //     }
  //     if(length > k_maxLength){
  //       extentPos = extentToExtentPos((k_maxLength + k_columToFront)/Math.cos(Math.toRadians( angle)));
  //       extent = extentPosToExtent(extentPos);
  //       height = extent * Math.sin(Math.toRadians( angle)) + k_pivotHeight;
  //       // angle = Math.toDegrees(Math.atan((height-k_pivotHeight)/k_maxLength));
  //       // leanglePos = angleToLeanglePos(angle);
  //     }
  //     System.out.println("height f"+ height);
  //     System.out.println("length f"+length);
  //     double leanglePosRatio = leanglePos/k_rangeLeanglePos*2-1;
  //     double extentPosRatio = extentPos/k_rangeExtentPos*2-1;

  //     setRaisePosition(angleR); 
  //     setExtendPositionArbFF(extentPosRatio);

  //     //raise is the angle of the arm
  
  // }
  
  public  void setArmPositionSafe(double angleR, double extentR){

    double expos = s_extend.mapInput(extentR);
    //pos max at horizontal =40
    // pos max at vert
    double downfudge = s_extend.m_Encoder.getVelocity()<0?5:0;

    // double fudge = getArmAngle()>0? getArmAngle()-10: getArmAngle()+5;
    double fudge = Math.min(Math.abs(leanglePosToAngle(s_raise.mapInput(angleR))) ,Math.abs( getAngle()));
    double maxExtent = (Units.inchesToMeters(39)+k_columToFront)/Math.abs(Math.cos(Math.toRadians(fudge)));
    double maxExtension =  (Units.inchesToMeters(70)+k_pivotHeight)/Math.abs(Math.sin(Math.toRadians(fudge)));
    double max = Math.min(maxExtension,maxExtent);
    
    double maxPos = extentToExtentPos(maxExtent);

    expos = Math.min(expos, maxPos);
    expos = expos/k_rangeExtentPos*2-1;


    expos = Math.min(expos,k_rangeExtentPos);
    setRaisePosition(angleR);
    setExtendPositionArbFF(expos);
  }
 

 //Arbituary FF 
 public void setExtendPositionArbFF(double position){
 position = s_extend.mapInput(position);
  double feedforward = s_extend.kFF_arb * Math.sin(Math.toRadians(getArmAngle())) + 0.25;
  vExtendSetPos= s_extend.setPosition(position, 0, feedforward);
    
  }

 

public void setExtendPosition(double position){
  //inpoutrange -1 to 
  position = s_extend.mapInput(position);

  if (position > vExtendSetPos){
  if (Math.abs(position-s_extend.getPosition() )< 3){
    position = s_extend.getPosition();
  }

  vExtendSetPos= s_extend.setPosition(position, 0);
}
  else
  vExtendSetPos= s_extend.setPosition(position, 1);
 
}


// public void setExtendPositionSafe(double position){
//   position = s_extend.mapInput(position);
//   // calculate max extension based on arm angle so it doesnt hit the floor
//   //max extion == k_pivotHeight/Math.cos(angle)
  
//   // final double k_horizontalPos = -50;//????? this is a guess, fill in with actual value
//   // final double k_minAngle = -30;//  ????? this is a guess, fill in with actual value by mesuring the angle of the arm
//   double angle = Math.toRadians( getArmAngle());
//   double maxExtension = position;
//   if (angle < 0){
//      maxExtension = ( k_pivotHeight-k_minArmHeight ) / Math.cos(angle);
//   }
//   maxExtension = (maxExtension-k_minLength)/(k_maxLength-k_minLength)*(k_rangeExtentPos);

//   if (position > maxExtension){
//     position = maxExtension;
//   }
//   vExtendSetPos=position;
//   vMaxExtention = maxExtension;


//   vExtendSetPos =  s_extend.setPosition(position ,0);
// }

public void setRaisePosition(double position){
  //inpoutrange -1 to 1
  //pos 0 is fully raised
  //negative pos is goint down 
  position = s_raise.mapInput(position);
  vRaisePos=s_raise.setPosition(position, 0);

}






//Claw setters
public void setClawPosition(double position){
  //inpoutrange -1 to 1
  //pos 0 is fully raised
  //negative pos is goint down 
  position = position +1;
  position = position/2;
  position = position * k_rangeClawPos ;
  vGripPos=position;
  m_gripPID.setReference(position, CANSparkMax.ControlType.kPosition);
  

}

public void toggleGripCone(){
  if(m_gripEncoder.getPosition()<38)
    setClawPosition(1);

  else 
    setClawPosition(-1);

}
public void toggleGripCube(){
  if(m_gripEncoder.getPosition()<5 || m_gripEncoder.getPosition()>30)
    setClawPosition(0.1);

  else 
    setClawPosition(-1);
}

///Motion seters
public void setExtendMotion(double position){
  //inpoutrange -1 to 1
  position = s_extend.mapInput(position);

  vExtendSetPos= s_extend.setMotion(position, 0);
}
public void setExtendMotionOnboard(double position){
  position = s_extend.mapInput(position);
  vExtendSetPos=s_extend.setMotionOB(position,0);
}

public void setRaiseMotion(double position){
  //inpoutrange -1 to 1
  //pos 0 is fully raised
  //negative pos is goint down 
 position = s_raise.mapInput(position);

  
  s_raise.setMotion(position, 0);
}

//reset encoders
public void resetPosition(){
  s_extend.setEncPosition(0);
  s_raise.setEncPosition(0);
  m_gripEncoder.setPosition(0);
}





//Kinematic helper methods
@Log.ToString(name = "Arm Pose", tabName = "Target Selector", rowIndex = 4, columnIndex = 6)
public Translation2d getArmPose(){

double armangle = getArmAngle();

double armlength = k_minLength + s_extend.getPosition()/k_rangeExtentPos * (k_maxLength-k_minLength);

double clawhieght = k_pivotHeight+ Math.sin(Math.toRadians(armangle))*armlength; 
double clawlength = Math.cos(Math.toRadians(armangle))*armlength-k_columToFront;




    Translation2d armpos = new Translation2d(clawlength, clawhieght);
  return armpos;
}
@Log(name = "Arm Angle")
public double getArmAngle(){
  double angle = 0;
  //use law of cosines because we have three sides
  //c^2 = a^2+b^2-2ab*cos(C)%
  //a^2+b^2 - c^2 = 2ab*cos(C)
  //C = acos((a^2+b^2 - c^2) /2ab)
  double a = k_pivotLenght;
  double b = k_pivotOffset;
  double c = k_maxLeangle+( s_raise.getPosition()/k_rangeLeanglePos *(k_maxLeangle-k_minLeangle));//negative positoning lead to subtracting from max angle
  // System.out.println("c: "+c);
  // System.out.println("a: "+a);
  // System.out.println("b: "+b);
  double temp = (Math.pow(a, 2)+Math.pow(b, 2)-Math.pow(c, 2))/(2*a*b);
  // System.out.println("temp: "+temp);
  angle = Math.toDegrees( Math.acos(temp)); // angle of obtuse triangle formed by arm
  angle = angle -90;// i of arm  above or below horizontal
  
 //calibrated fudging
  double measuredmin = -23.8;
  double measuredmax = 62;
  double calcmin =-26.9;
  double calcmax = 52.4;
  double zerooffset = -5.27;


  angle =   (angle-calcmin)/(calcmax-calcmin)*(measuredmax-measuredmin)+measuredmin; 


  return angle;
}
public double leanglePosToAngle(double anglePos){
  double angle = 0;
  //use law of cosines because we have three sides
  //c^2 = a^2+b^2-2ab*cos(C)%
  //a^2+b^2 - c^2 = 2ab*cos(C)
  //C = acos((a^2+b^2 - c^2) /2ab)
  double a = k_pivotLenght;
  double b = k_pivotOffset;
  double c = k_maxLeangle+( anglePos/k_rangeLeanglePos *(k_maxLeangle-k_minLeangle));//negative positoning lead to subtracting from max angle
  // System.out.println("c: "+c);
  // System.out.println("a: "+a);

  // System.out.println("b: "+b);
  double temp = (Math.pow(a, 2)+Math.pow(b, 2)-Math.pow(c, 2))/(2*a*b);
  // System.out.println("temp: "+temp);
  angle = Math.toDegrees( Math.acos(temp)); // angle of obtuse triangle formed by arm
  angle = angle -90;// i of arm  above or below horizontal
  
 //calibrated fudging
  double measuredmin = -23.8;
  double measuredmax = 62;
  double calcmin =-26.9;
  double calcmax = 52.4;
  double zerooffset = -5.27;


  angle =   (angle-calcmin)/(calcmax-calcmin)*(measuredmax-measuredmin)+measuredmin; 

  // System.out.println("angle "+angle);
  return angle;
}

public double getArmLength(){
  double length = 0;
  //use law of cosines because we have three sides
  //c^2 = a^2+b^2-2ab*cos(C)
  //a^2+b^2 - c^2 = 2ab*cos(C)
  //C = acos((a^2+b^2 - c^2) /2ab)
  // double a = k_pivotLenght;
  // double b = k_pivotOffset;
  // double c = m_raiseEncoder.getPosition()/k_rangeLena*(k_maxLeangle-k_minLeangle)+k_minLeangle;
  double extent = s_extend.getPosition()/k_rangeExtentPos*(k_maxLength-k_minLength)+k_minLength;
  length = extent * Math.cos(getArmAngle())- k_columToFront;
  return length;

}



public double angleToLeanglePos(double angle){
  //TAKES DEGREES
  //angle is the angle of the arm above or below horizontal
 //use law of cosines because we have 2 sides and an included angle
  //c^2 = a^2+b^2-2ab*cos(C)
  // solve for c 
  //C =  the  moving angle of the arm
double measuredmin = -23.8;
double measuredmax = 62;
double calcmin =-26.9;
double calcmax = 52.4;
double zerooffset = -5.27;


  angle =   (angle-measuredmin)/(calcmax-calcmin)*(measuredmax-measuredmin)+calcmin; 




  double C = angle + 90;
  C = Math.toRadians(angle);
  double a = k_pivotLenght;
  double b = k_pivotOffset;
  double c = Math.sqrt(Math.pow(a, 2)+Math.pow(b, 2)-2*a*b*Math.cos(C));
  double leangle = (c-k_minLeangle)/(k_maxLeangle-k_minLeangle)*k_maxLeanglePos*-1;
  return leangle;


}


public double extentToExtentPos(double extent){
  //extent is the length of the arm
  extent = extent ;
  double extentPos = (extent-k_minLength)/(k_maxLength-k_minLength)*k_rangeExtentPos;
  return extentPos;
}
public double extentPosToExtent(double extentPos){
  //extent is the length of the arm
  double extent = (extentPos/k_rangeExtentPos)*(k_maxLength-k_minLength)+k_minLength;

  return extent;
}


public Translation2d radialToLengths(double angle, double length){
 //calculate the lengths of the arm and angle motor to get to the desired position
 //angle is the angle of the arm above or below horizontal
 //length is the distance from the front of the robot to the desired position

 double leanglePos = angleToLeanglePos(angle);
 double extent = (length + k_columToFront) /Math.cos(angle);
 double extentPos = extentToExtentPos(extent);
 

  return new Translation2d(leanglePos,extentPos);
}

public Translation2d cartToPosRatio(double length, double height){
  //calculate the lengths of the arm and angle motor to get to the desired position
  //height is the hieght of the desired position
  //length is the distance from the front of the robot to the desired position
  double Y= height - k_pivotHeight;
  double X = length + k_columToFront;
  double angle = Math.toDegrees(Math.atan(Y/X));
  double leanglePos = angleToLeanglePos(angle);
  double leanglePosRatio = leanglePos/k_rangeLeanglePos*2-1;
  
 
  double extent = X/Math.cos(Math.toRadians(angle));
  
  double lengthPos = extentToExtentPos(extent);
  double lengthPosRatio = lengthPos/k_rangeExtentPos*2-1;
 
  
 
   return new Translation2d(leanglePosRatio,lengthPosRatio);
 }

//simple commads
public Command dropCargo(Translation2d pose){
  return new SequentialCommandGroup(
   
    new InstantCommand(()->setArmPositionSafe(0.45, 0.65)),//slider values
    new WaitCommand(1),
    new InstantCommand(()->setClawPosition(1)),
    new WaitCommand(1),
    new InstantCommand(()->setClawPosition(0)),
    new InstantCommand(()->setArmPositionSafe(1,-1))
    //slider values
  );
}
public Command dropCargo(){
  return new SequentialCommandGroup(
   
    new InstantCommand(()->setArmPositionSafe(-0.45, 0.75)),//slider values
    new WaitCommand(1),
    new InstantCommand(()->setClawPosition(0)),
    new WaitCommand(2),
    // new InstantCommand(()->setClawPosition(1)),
    new InstantCommand( () -> setArmPositionSafe(-1,0.75)),
    new WaitCommand(2),
    new InstantCommand(()->setArmPositionSafe(-1,-1)),
    new WaitCommand(2)
    );
    //slider values
  
}
public Command dropCargo2(){
  return new SequentialCommandGroup(
   
    new InstantCommand(()->setArmPositionSafe(-0.45, 0.75),this),//slider values
    new WaitCommand(1),
    new InstantCommand(()->setClawPosition(-1)),
    new WaitCommand(1),
    // new InstantCommand(()->setClawPosition(1)),
    new InstantCommand( () -> setArmPositionSafe(-1,0.2),this),
    new WaitCommand(1),
    new InstantCommand(()->setArmPositionSafe(-1,-1),this),
    new WaitCommand(1)
    );
    //slider values


  
}



// Data Helper Methods
@Log(name = "Extent Position")
public double getExtendedPosition(){
  return s_extend.getPosition();
}

// @Log(name = "Extent")
public double getExtent(){
  return s_extend.getPosition()/k_rangeExtentPos*(k_maxLength-k_minLength)+k_minLength;
}

// @Log(name = "Raise Position")
public double getRaisedPosition(){
  return s_raise.getPosition();
}
// 
// @Log(name = "Grip Position")
public double getGripPosition(){
  return m_gripEncoder.getPosition();
}

@Log(name = "Grip Current")
public double getGripCurrent(){
  return m_gripMotor.getOutputCurrent();
}

//more arbff tuning methods
@Log(name = "Arm Angle", tabName = "Extend FF", rowIndex = 2, columnIndex = 0)
public double getAngle(){
  return getArmAngle();
}
// @Log(name = "karbFF", tabName = "Extend FF", rowIndex = 1, columnIndex = 0)
public double getKarbFF(){
  return s_extend.getMaxVoltage()/Math.sin(Math.toRadians(getArmAngle()));
}

// @Log(name = "Max_karbFF", tabName = "Extend FF", rowIndex = 1, columnIndex = 1)
public double getMaxKarbFF(){
  return s_extend.getMaxVoltage()/Math.sin(Math.toRadians(getArmAngle()));
}

// @Log(name = "Max_karbFF_stoped", tabName = "Extend FF", rowIndex = 1, columnIndex = 2)
public double getMaxKarbFFStoped(){
  return s_extend.getMaxVoltageStopped()/Math.sin(Math.toRadians(getArmAngle()));
}





//oblog config methods
@Config (name = "Grip Max Output", tabName = "Arm")
void setGripMaxOutput(@Config(defaultValueNumeric = kMaxOutputG) double gripPower, @Config(defaultValueNumeric = kMinOutputG) double releasePower){
  
  m_gripPID.setOutputRange(-gripPower, releasePower);
}

 

// @Config.NumberSlider (name = "min height", tabName = "Arm", min = 0, max = 8, defaultValue = d_minArmHeight)
void setMinArmHeight( double minHeight){
  k_minArmHeight = minHeight;



}
// @Log.Graph(name = "Extend Setpoint")
// double vExtendPos(){
//   return vExtendSetPos;
// }
// @Log.Graph(name = "Raise Setpoint")
// double vRaisePos(){
//   return vRaisePos;
// }
// @Log.Graph(name = "Grip Setpoint")
// double vGripPos(){
//   return vGripPos;
// }


  // @Log.Graph (name = "Raise Error", tabName = "Arm PID")
  // public double getRaiseError(){
  //   return m_raiseEncoder.getPosition() - vRaisePos;
  // }
  // @Log.Graph (name = "Grip Error", tabName = "Arm PID")
  // public double getGripError(){
  //   return m_gripEncoder.getPosition() - vGripPos;
  // }



}