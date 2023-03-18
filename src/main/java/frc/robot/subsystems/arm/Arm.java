package frc.robot.subsystems.arm;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import io.github.oblarg.oblog.Loggable;
import io.github.oblarg.oblog.annotations.Config;
import io.github.oblarg.oblog.annotations.Log;
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
  
public final CANSparkMax m_extendMotor;
public final CANSparkMax m_raiseMotor;
public final CANSparkMax m_gripMotor;

RelativeEncoder  m_extendEncoder ;
RelativeEncoder m_raiseEncoder ;
RelativeEncoder m_gripEncoder;

SparkMaxPIDController m_extendPID;
SparkMaxPIDController m_raisePID;
SparkMaxPIDController m_gripPID;


private final double k_maxLengthPos = 38.3;//postion at the max length we mesures in units
private final double k_minLengthPos = 0;//postion at the max length we mesures in units



private final double k_maxLeanglePos = 116 ; //max length of the angle control in units
private final double k_minLeanglePos = 0 ; //max length of the angle control in units
private final double k_maxClawPos = 33.3; //43.3
private final double k_minClawPos = -10;

// private final double k_rangeLengthPos = k_maxLengthPos - k_minLengthPos;
// private final double k_rangeLeanglePos = k_maxLeanglePos- k_minLeanglePos;
private final double k_rangeLengthPos = 38.3;
private final double k_rangeLeanglePos =116;//needs to be tuned, adjust tilll the arm is at the right angle at bottom should be nearly correct

private final double k_rangeClawPos = k_maxClawPos - k_minClawPos; //also should be checked and tuned


private final double k_maxHeight = Units.inchesToMeters((12*6.25)) ;// height in feet that the robot is alowed to be

private final double k_pivotHeight = Units.inchesToMeters(22); // height in inches that the pivot point oi the arm is of the floor
private double k_minArmHeight = Units.inchesToMeters(0); // height in inches that the arm is alowed to be at the bottom
private final double d_minArmHeight = 5;


private final double k_defaultLength = 0; // the length we want the ar m to be most of the time 
private final double k_defaultLeanglle= 0;//the lenght of the angle  motor we want most of the time 

private final double k_coneClaw=0;
private final double k_cubeClaw=0;

private final double k_coneLeangle1= 0 ;
private final double k_coneLeangle2= 0 ;
private final double k_coneLeangle3= 0 ;
private final double k_coneLeangleS= 0 ;

private final double k_cubeLeangle1= 0 ;
private final double k_cubeLeangle2= 0 ;
private final double k_cubeLeangle3= 0 ;
private final double k_cubeLeangleS= 0 ;


private final double k_coneLength1= 0 ;
private final double k_coneLength2= 0 ;
private final double k_coneLength3= 0 ;
private final double k_coneLengthS= 0 ;

private final double k_cubeLength1= 0 ;
private final double k_cubeLength2= 0 ;
private final double k_cubeLength3= 0 ;
private final double k_cubeLengthS= 0 ;


// private final double k_pivotLenght = Units.inchesToMeters(17.5);//? the hieght of the arm pivot from the base of the angle actuator fill in actual
private final double k_pivotLenght = Units.inchesToMeters(16.5);
private final double k_pivotOffset = Units.inchesToMeters(5.25); //distance between the arm pirvot point and the place where the angle motor attaches to the arm, 
// private final double k_minLeangle= Units.inchesToMeters(13.4);//used for kinematics
private final double k_minLeangle= Units.inchesToMeters(14.87);
// private final double k_maxLeangle= Units.inchesToMeters(21.2);//used for kinemati
private final double k_maxLeangle= Units.inchesToMeters(21.0);
private final double k_minLength= Units.inchesToMeters(30.75);
private final double k_maxLength= Units.inchesToMeters(59);
private final double k_columtToFront = Units.inchesToMeters(18); //distance of the colloum from the front of the robot

private final double k_ticksPerInchExtend=1;
private final double k_ticksPerInchRaise=1;
private final double k_ticksPerInchGrip= 1;

  //pid constants
  //extend
 public final double kPE = 1.0;//0.14
 public final double kIE = 0.00;//was 0.05
 public final double kDE = 0.001;//was 0.01
 public final double kIzE = 2;
 public final double kFFE = 0.0;
 public final double kMaxOutputE = 0.4; //arm oout?
 public final double kMinOutputE = 0.3;//arm in?
 public final double maxRPME = 5;
 public final double maxAccelE = 2;

//raise
  public final double kPR = 0.6;//0.6
  public final double kIR = 0.000;//was 0.0005
  public final double kDR = 0.001;//was 0.001
  public final double kIzR = 2;
  public final double kFFR = 0.0;
  public final double kMaxOutputR = 0.4;//arm up
  public final double kMinOutputR = 0.3;//arm down
  
  public final double maxRPMR = 5;
  public final double maxAccelR = 2;


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

  
  public double vExtendPos = 0;

  public double vRaisePos = 0;
  public double vGripPos = 0;

  @Log.Graph(name = "minArmHeight")
  public double vMaxExtention = 0;
  @Log.Graph(name = "minAgnle")
  public double vMinAngle = 0;
  public double k_targetRaisePosHigh;
  public double k_targetExtPosHigh;
  public double k_targetGripPosHigh;


// private final AbsoluteEncoder m_Encoder;




  public Arm(){
  m_extendMotor = new CANSparkMax(20, MotorType.kBrushless);
   m_raiseMotor = new CANSparkMax(21,MotorType.kBrushless);
   m_gripMotor = new CANSparkMax(22,MotorType.kBrushless) ;
   
   //cuewnt limit
   m_extendMotor.setSmartCurrentLimit(50 , 50,0); 
   m_raiseMotor.setSmartCurrentLimit(10,15,0);
   m_gripMotor.setSmartCurrentLimit(6,8,0);

   m_extendMotor.setSoftLimit(SoftLimitDirection.kForward, (float)k_rangeLengthPos);
    m_extendMotor.setSoftLimit(SoftLimitDirection.kReverse, (float)-1);

    m_raiseMotor.setSoftLimit(SoftLimitDirection.kForward, (float) 1);
    m_raiseMotor.setSoftLimit(SoftLimitDirection.kReverse, (float)-k_rangeLeanglePos);

    


   // neutral mode
   m_extendMotor.setIdleMode(IdleMode.kBrake);
   m_raiseMotor.setIdleMode(IdleMode.kBrake);
   m_gripMotor.setIdleMode(IdleMode.kBrake);


  //  m_extendMotor.setIdleMode(IdleMode.kCoast);
  //  m_raiseMotor.setIdleMode(IdleMode.kCoast);
  //  m_gripMotor.setIdleMode(IdleMode.kCoast);

    // m_extendMotor.setSoftLimit(SoftLimitDirection.kForward, (float)k_rangeLengthPos);
    // m_raiseMotor.setSoftLimit(SoftLimitDirection.kReverse, (float)-k_rangeLeanglePos);

    // m_extendMotor.setSoftLimit(SoftLimitDirection.kReverse, 0);
    // m_raiseMotor.setSoftLimit(SoftLimitDirection


    
   

    
  //encoder
    m_extendEncoder = m_extendMotor.getEncoder();
    m_raiseEncoder = m_raiseMotor.getEncoder();
    m_gripEncoder = m_gripMotor.getEncoder();


    m_extendEncoder.setPositionConversionFactor(k_ticksPerInchExtend);
    m_raiseEncoder.setPositionConversionFactor(k_ticksPerInchRaise );
    m_gripEncoder.setPositionConversionFactor(k_ticksPerInchGrip);


// pid controllers
    m_extendPID = m_extendMotor.getPIDController();
    m_raisePID = m_raiseMotor.getPIDController();
    m_gripPID = m_gripMotor.getPIDController();




    // m_extendPID.setP(5.0);
    // m_extendPID.setI(0.05);
    // m_extendPID.setD(0.01);
    // m_extendPID.setIZone(2);
    // m_extendPID.setFF(0);
    // m_extendPID.setOutputRange(-0.3, 0.8);
    m_extendPID.setP(kPE,0);
    m_extendPID.setI(kIE,0);
    m_extendPID.setD(kDE,0);
    m_extendPID.setIZone(kIzE,0);
    m_extendPID.setFF(kFFE,0);
    m_extendPID.setOutputRange(-kMinOutputE, kMaxOutputE,0);
    



    // m_raisePID.setP(0.05);
    // m_raisePID.setI(0.0005);
    // m_raisePID.setD(0.001);
    // m_raisePID.setIZone(2);
    // m_raisePID.setFF(0);
    // m_raisePID.setOutputRange(-0.3, 0.8);
    m_raisePID.setP(kPR,0);
    m_raisePID.setI(kIR,0);
    m_raisePID.setD(kDR,0);
    m_raisePID.setIZone(kIzR,0);
    m_raisePID.setFF(kFFR,0);
    m_raisePID.setOutputRange(-kMinOutputR, kMaxOutputR,0);
    
    // m_gripPID.setP(1);
    // m_gripPID.setI(0);
    // m_gripPID.setD(0);
    // m_gripPID.setIZone(0);
    // m_gripPID.setFF(0);
    // m_gripPID.setOutputRange(-0.4, 0.4);
    m_gripPID.setP(kPG,0);
    m_gripPID.setI(kIG,0);
    m_gripPID.setD(kDG,0);
    m_gripPID.setIZone(kIzG,0);
    m_gripPID.setFF(kFFG,0);
    m_gripPID.setOutputRange(-kMinOutputG, kMaxOutputG,0);

    // motion contfiguration
    int kSlotIdxE = 0;
    m_extendPID.setSmartMotionMaxVelocity(maxRPME, kSlotIdxE);
    m_extendPID.setSmartMotionMinOutputVelocity(-maxRPME, kSlotIdxE);
    m_extendPID.setSmartMotionMaxAccel(maxAccelE, kSlotIdxE);
    m_extendPID.setSmartMotionAllowedClosedLoopError(10, kSlotIdxE);

    int kSlotIdxR = 0;
    m_raisePID.setSmartMotionMaxVelocity(maxRPMR, kSlotIdxR);
    m_raisePID.setSmartMotionMinOutputVelocity(-maxRPMR, kSlotIdxR);
    m_raisePID.setSmartMotionMaxAccel(maxAccelR, kSlotIdxR);
    m_raisePID.setSmartMotionAllowedClosedLoopError(10, kSlotIdxR);

    int kSlotIdxG = 0;
    m_gripPID.setSmartMotionMaxVelocity(maxRPMG, kSlotIdxG);
    m_gripPID.setSmartMotionMinOutputVelocity(-maxRPMG, kSlotIdxG);
    m_gripPID.setSmartMotionMaxAccel(maxAccelG, kSlotIdxG);
    m_gripPID.setSmartMotionAllowedClosedLoopError(10, kSlotIdxG);



    //set some soft limit to prevent full extension (illegal)
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    // checkIfStalled();
  }

  // @Log(name = "Pivot Encoder Counts", tabName = "Arm")
  // public double getAbsoluteEncoderCounts(){
  //   return m_Encoder.getPosition();
  // }

  // @Log(name = "Check if Stalled")
  // public boolean checkIfStalled(){
  //   //Check pulse per 10 ms. If the rate of pulse immediately drops, then we are stalled.
  //   boolean isStalled = (m_armMotor.getOutputCurrent() < Constants.voltageDropThreshold);
  //   if(isStalled){
  //     m_armMotor.setSmartCurrentLimit(0, 0); //TBD
  //   }
  //   return isStalled;
  // }

  @Log.Graph(name = "Arm Motor Pulse")
  public double getMotorPulse(){
    return m_extendMotor.getOutputCurrent();
  }

@Log.ToString(name = "Arm Pose")
public Translation2d getArmPose(){

double armangle = getArmAngle();

double armlength = k_minLength + m_extendEncoder.getPosition()/k_rangeLengthPos * (k_maxLength-k_minLength);

double clawhieght = k_pivotHeight+ Math.sin(armangle)*armlength; 
double clawlength = Math.cos(armangle)*armlength-k_columtToFront;




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
  double c = k_maxLeangle+( m_raiseEncoder.getPosition()/k_rangeLeanglePos *(k_maxLeangle-k_minLeangle));//negative positoning lead to subtracting from max angle
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

public double getArmLength(){
  double length = 0;
  //use law of cosines because we have three sides
  //c^2 = a^2+b^2-2ab*cos(C)
  //a^2+b^2 - c^2 = 2ab*cos(C)
  //C = acos((a^2+b^2 - c^2) /2ab)
  // double a = k_pivotLenght;
  // double b = k_pivotOffset;
  // double c = m_raiseEncoder.getPosition()/k_rangeLena*(k_maxLeangle-k_minLeangle)+k_minLeangle;
  double extent = m_extendEncoder.getPosition()/k_rangeLengthPos*(k_maxLength-k_minLength)+k_minLength;
  length = extent * Math.cos(getArmAngle())- k_columtToFront;
  return length;

}



public double angleToLeangle(double angle){
  //angle is the angle of the arm above or below horizontal
 //use law of cosines because we have 2 sides and an included angle
  //c^2 = a^2+b^2-2ab*cos(C)
  // solve for c 
  //C =  the  moving angle of the arm
  double C = angle + 90;
  double a = k_pivotLenght;
  double b = k_pivotOffset;
  double c = Math.sqrt(Math.pow(a, 2)+Math.pow(b, 2)-2*a*b*Math.cos(C));
  double leangle = (c-k_minLeangle)/(k_maxLeangle-k_minLeangle);
  return leangle;


}

public Translation2d radialToLengths(double angle, double length){
 //calculate the lengths of the arm and angle motor to get to the desired position
 //angle is the angle of the arm above or below horizontal
 //length is the distance from the front of the robot to the desired position

 double leangle = angleToLeangle(angle);
 length = length/Math.cos(angle) - k_columtToFront;

 

  return new Translation2d(leangle,length);
}

public Translation2d cartToLengths(double length, double height){
  //calculate the lengths of the arm and angle motor to get to the desired position
  //height is the hieght of the desired position
  //length is the distance from the front of the robot to the desired position
  height = height - k_pivotHeight;
  length = length + k_columtToFront;
  double angle = Math.atan(height/length);
  double leangle = angleToLeangle(angle);
  length = length/Math.cos(angle);
 
  
 
   return new Translation2d(leangle,length);
 }

@Log(name = "Extent Position")
public double getExtendedPosition(){
  return m_extendEncoder.getPosition();
}

@Log(name = "Extent")
public double getExtent(){
  return m_extendEncoder.getPosition()/k_rangeLengthPos*(k_maxLength-k_minLength)+k_minLength;
}

@Log(name = "Raise Position")
public double getRaisedPosition(){
  return m_raiseEncoder.getPosition();
}
@Log(name = "Grip Position")
public double getGripPosition(){
  return m_gripEncoder.getPosition();
}
 public void setArmPosition(Translation2d position){
    //takes in a pose and sets the arm to that position
    Translation2d setpoints = cartToLengths(position.getX(), position.getY());

    setExtendPosition(setpoints.getY());
    setRaisePosition(setpoints.getX());

    //  setExtendMotion(setpoints.getY());
    //  setRaiseMotion(setpoints.getX());  

    //   setExtendMotionSafe(setpoints.getY());
    //   setRaiseMotionSafe(setpoints.getX());

 }
 public Command dropCargo(Translation2d position){
   return new SequentialCommandGroup(
     new InstantCommand(()->setClawPosition(0)),
     new InstantCommand(()->setArmPosition(position)),
     new WaitCommand(0.5),
     new InstantCommand(()->setClawPosition(1)),
     new WaitCommand(0.5),
     new InstantCommand(()->setClawPosition(0)),
     new InstantCommand(()->setArmPosition(new Translation2d(0,0)))
   );
 }


public void setExtendPosition(double position){
  //inpoutrange -1 to 1
  // pos zero is fully retracted
  //1 is fully extended
  position = position +1;
  position = position/2;
  position = position * (k_rangeLengthPos-2);
  position = position+2;
  vExtendPos=position;
  m_extendPID.setReference(position, CANSparkMax.ControlType.kPosition);
}
public void setRaisePosition(double position){
  //inpoutrange -1 to 1
  //pos 0 is fully raised
  //negative pos is goint down 
  position = position +1;
  position = position/2;
  position = position * (k_rangeLeanglePos-2) *-1;
  position = position -2;
  vRaisePos=position;

  m_raisePID.setReference(position, CANSparkMax.ControlType.kPosition);


}
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

public void setExtendMotion(double position){
  //inpoutrange -1 to 1
  // pos zero is fully retracted
  //1 is fully extended
  position = position +1;
  position = position/2;
  position = position * (k_rangeLengthPos-2);
  position = position+2;
  vExtendPos=position;
  m_extendPID.setReference(position, CANSparkMax.ControlType.kSmartMotion,0);
}
public void setRaiseMotion(double position){
  //inpoutrange -1 to 1
  //pos 0 is fully raised
  //negative pos is goint down 
  position = position +1;
  position = position/2;
  position = position * (k_rangeLeanglePos-2) *-1;
  position = position -2;
  vRaisePos=position;
  m_raisePID.setReference(position, CANSparkMax.ControlType.kSmartMotion,0);
}
public void setClawMotion(double position){
  //inpoutrange -1 to 1
  //pos 0 is fully raised
  //negative pos is goint down 
  position = position +1;
  position = position/2;
  position = position * k_rangeClawPos ;
  vGripPos=position;
  m_gripPID.setReference(position, CANSparkMax.ControlType.kSmartMotion,0);
  

}
public void setExtendMotionSafe(double position){
  //inpoutrange -1 to 1
  // pos zero is fully retracted
  //1 is fully extended
  position = position +1;
  position = position/2;
  position = position * (k_rangeLengthPos-2);
  position = position+2;
  // calculate max extension based on arm angle so it doesnt hit the floor
  //max extion == k_pivotHeight/Math.cos(angle)
  
  // final double k_horizontalPos = -50;//????? this is a guess, fill in with actual value
  // final double k_minAngle = -30;//  ????? this is a guess, fill in with actual value by mesuring the angle of the arm
  double angle = getArmAngle();
  double maxExtension = position;
  if (angle < 0){
     maxExtension = ( k_pivotHeight-k_minArmHeight ) / Math.cos(angle);
  }
  maxExtension = (maxExtension-k_minLength)/(k_maxLength-k_minLength)*(k_rangeLengthPos);

  if (position > maxExtension){
    position = maxExtension;
  }
  vExtendPos=position;
  vMaxExtention = maxExtension;

  m_extendPID.setReference(position, CANSparkMax.ControlType.kSmartMotion);
}

public void setRaiseMotionSafe( double position){
  //inpoutrange -1 to 1
  //pos 0 is fully raised
  //negative pos is goint down 
  position = position +1;
  position = position/2;
  position = position * (k_rangeLeanglePos-2) *-1;
  position = position -2;
  // calcuate min angle/leangle based on arm extension so it doesnt hit the floor
  
  double lenght = getExtent();
 // min angle = Math.cos(angle)= k_pivotHeight/length
  double minAngle = -Math.acos((k_pivotHeight-k_minArmHeight)/lenght);
  minAngle = angleToLeangle(minAngle)*k_rangeLeanglePos;
  
  if (position < minAngle){
    position = minAngle;
  }
  vRaisePos=position;
  vMinAngle = minAngle;

  m_raisePID.setReference(position, CANSparkMax.ControlType.kSmartMotion);

}


public void resetPosition(){
  m_extendEncoder.setPosition(0);
  m_raiseEncoder.setPosition(0);
  m_gripEncoder.setPosition(0);
}


 
// @Config(name = "Extend PID", tabName = "Arm PID")
// void setExtenPIDIzF(@Config(defaultValueNumeric = kPE) double p , @Config(defaultValueNumeric = kIE) double i, @Config(defaultValueNumeric = kDE) double d, @Config(defaultValueNumeric = kIzE) double iz, @Config(defaultValueNumeric = kFFE) double f){  
//   m_extendPID.setP(p);
//   m_extendPID.setI(i);
//   m_extendPID.setD(d);
//   m_extendPID.setIZone(iz);
//   m_extendPID.setFF(f);

// }
@Config (name = "Extend P", tabName = "Arm PID", defaultValueNumeric = kPE)
void setExtendP(double p){
  m_extendPID.setP(p);
}
@Config (name = "Extend I", tabName = "Arm PID", defaultValueNumeric = kIE)
void setExtendI(double i){
  m_extendPID.setI(i);
}
@Config (name = "Extend D", tabName = "Arm PID", defaultValueNumeric = kDE)
void setExtendD(double d){
  m_extendPID.setD(d);
}
@Config (name = "Extend Iz", tabName = "Arm PID", defaultValueNumeric = kIzE)
void setExtendIz(double iz){
  m_extendPID.setIZone(iz);
}
@Config (name = "Extend FF", tabName = "Arm PID", defaultValueNumeric = kFFE)
void setExtendFF(double f){
  m_extendPID.setFF(f);
}


// @Config (name = "Raise PID", tabName = "Arm PID")
// void setRaisePIDIzF(@Config(defaultValueNumeric = kPR) double p , @Config(defaultValueNumeric = kIR) double i, @Config(defaultValueNumeric = kDR) double d, @Config(defaultValueNumeric = kIzR) double iz, @Config(defaultValueNumeric = kFFR) double f) {
//   m_raisePID.setP(p);
//   m_raisePID.setI(i);
//   m_raisePID.setD(d);
//   m_raisePID.setIZone(iz);
//   m_raisePID.setFF(f);
// }
@Config (name = "Raise P", tabName = "Arm PID", defaultValueNumeric = kPR)
void setRaiseP(double p){
  m_raisePID.setP(p);
}
@Config (name = "Raise I", tabName = "Arm PID", defaultValueNumeric = kIR)
void setRaiseI(double i){
  m_raisePID.setI(i);
}
@Config (name = "Raise D", tabName = "Arm PID", defaultValueNumeric = kDR)
void setRaiseD(double d){
  m_raisePID.setD(d);
}
@Config (name = "Raise Iz", tabName = "Arm PID", defaultValueNumeric = kIzR)
void setRaiseIz(double iz){
  m_raisePID.setIZone(iz);
}
@Config (name = "Raise FF", tabName = "Arm PID", defaultValueNumeric = kFFR)
void setRaiseFF(double f){
  m_raisePID.setFF(f);
}


// @Config (name = "Grip PID", tabName = "Arm PID")
// void setGripPIDIzF( @Config( defaultValueNumeric = kPG) double p , @Config(defaultValueNumeric = kIG) double i, @Config(defaultValueNumeric = kDG) double d, @Config(defaultValueNumeric = kIzG) double iz, @Config(defaultValueNumeric = kFFG) double f) {
//   m_gripPID.setP(p);
//   m_gripPID.setI(i);
//   m_gripPID.setD(d);
//   m_gripPID.setIZone(iz);
//   m_gripPID.setFF(f);
// }

// @Config (name = "Extend Max Motion", tabName = "Arm PID")
// void setExtendMaxVelocityAndAccel(@Config(defaultValueNumeric = maxRPME) double maxVelocity, @Config(defaultValueNumeric = maxAccelE) double maxAccel){
//   m_extendPID.setSmartMotionMaxVelocity(maxVelocity, 0);
//   m_extendPID.setSmartMotionMaxAccel(maxAccel, 0);
// }

// @Config (name = "Raise Max Motion", tabName = "Arm PID")
// void setRaiseMaxVelocityAndAccel(@Config(defaultValueNumeric = maxRPMR) double maxVelocity, @Config(defaultValueNumeric = maxAccelR) double maxAccel){
//   m_raisePID.setSmartMotionMaxVelocity(maxVelocity, 0);
//   m_raisePID.setSmartMotionMaxAccel(maxAccel, 0);
// }

// @Config (name = "Grip Max Motion", tabName = "Arm PID")
// void setGripMaxVelocityAndAccel(@Config(defaultValueNumeric = maxRPMG) double maxVelocity, @Config(defaultValueNumeric = maxAccelG) double maxAccel){
//   m_gripPID.setSmartMotionMaxVelocity(maxVelocity, 0);
//   m_gripPID.setSmartMotionMaxAccel(maxAccel, 0);
// }


@Config (name = "Extend Max Output", tabName = "Arm PID")
void setExtendMaxOutput(@Config(defaultValueNumeric = kMaxOutputE) double outPower, @Config(defaultValueNumeric = kMinOutputE) double inPower){
  
  m_extendPID.setOutputRange(-inPower, outPower);
}


@Config (name = "Raise Max Output", tabName = "Arm PID")
void setRaiseMaxOutput(@Config(defaultValueNumeric = kMinOutputR) double upPower, @Config(defaultValueNumeric = kMaxOutputR) double downPower){
  
  m_raisePID.setOutputRange(-upPower, downPower);
}

@Config (name = "Grip Max Output", tabName = "Arm PID")
void setGripMaxOutput(@Config(defaultValueNumeric = kMaxOutputG) double gripPower, @Config(defaultValueNumeric = kMinOutputG) double releasePower){
  
  m_gripPID.setOutputRange(-gripPower, releasePower);
}

 

@Config.NumberSlider (name = "min height", tabName = "Arm PID", min = 0, max = 8, defaultValue = d_minArmHeight)
void setMinArmHeight( double minHeight){
  k_minArmHeight = minHeight;



}
@Log.Graph(name = "Extend Setpoint")
double vExtendPos(){
  return vExtendPos;
}
@Log.Graph(name = "Raise Setpoint")
double vRaisePos(){
  return vRaisePos;
}
@Log.Graph(name = "Grip Setpoint")
double vGripPos(){
  return vGripPos;
}

@Log.Graph (name = "Extend Error", tabName = "Arm PID")
  public double getExtendError(){
    return m_extendEncoder.getPosition() - vExtendPos;
  }
  @Log.Graph (name = "Raise Error", tabName = "Arm PID")
  public double getRaiseError(){
    return m_raiseEncoder.getPosition() - vRaisePos;
  }
  @Log.Graph (name = "Grip Error", tabName = "Arm PID")
  public double getGripError(){
    return m_gripEncoder.getPosition() - vGripPos;
  }



}