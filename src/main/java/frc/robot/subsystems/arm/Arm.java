package frc.robot.subsystems.arm;
import frc.robot.subsystems.arm.Extend;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
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
final static double k_rangeLengthPos = 55;
final static double k_rangeLeanglePos =116;//needs to be tuned, adjust tilll the arm is at the right angle at bottom should be nearly correct

private final double k_rangeClawPos = k_maxClawPos - k_minClawPos; //also should be checked and tuned


private final double k_maxHeight = Units.inchesToMeters((12*6.25)) ;// height in feet that the robot is alowed to be

private final double k_pivotHeight = Units.inchesToMeters(21); // height in inches that the pivot point oi the arm is of the floor
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
private final double k_maxLength= Units.inchesToMeters(70);//59
private final double k_columtToFront = Units.inchesToMeters(18); //distance of the colloum from the front of the robot

final static double k_ticksPerInchExtend=1;
private final double k_ticksPerInchRaise=1;
private final double k_ticksPerInchGrip= 1;

  //pid constants
  

//raise
  public final double kPR_up = 0.06;//0.6
  public final double kIR_up = 0.0001;//was 0.0005
  public final double kDR_up = 0.0000;//was 0.001
  public final double kIzR_up = 2;
  public final double kFFR_up = 0.0;

  public final double kPR_down = 0.06;//0.6
  public final double kIR_down = 0.0001;//was 0.0005
  public final double kDR_down = 0.0000;//was 0.001
  public final double kIzR_down = 2;
  public final double kFFR_down = 0.0;

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

  
  public double vExtendSetPos = 0;

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



//Sets max velocity, acceleration, and state
  private TrapezoidProfile armSmother = new TrapezoidProfile(new TrapezoidProfile.Constraints(0,0),
                                                             new TrapezoidProfile.State(0, 0), 
                                                             new TrapezoidProfile.State(0,0));
  public Arm(){
    s_extend = new Extend();
    s_raise = new Raise();

   m_gripMotor = new CANSparkMax(22,MotorType.kBrushless) ;
   
   //cuewnt limit
   
   m_gripMotor.setSmartCurrentLimit(16,28,0);



    


   // neutral mode
   
   
   m_gripMotor.setIdleMode(IdleMode.kBrake);


  //  m_extendMotor.setIdleMode(IdleMode.kCoast);
  //  m_raiseMotor.setIdleMode(IdleMode.kCoast);
  //  m_gripMotor.setIdleMode(IdleMode.kCoast);

    // m_extendMotor.setSoftLimit(SoftLimitDirection.kForward, (float)k_rangeLengthPos);
    // m_raiseMotor.setSoftLimit(SoftLimitDirection.kReverse, (float)-k_rangeLeanglePos);

    // m_extendMotor.setSoftLimit(SoftLimitDirection.kReverse, 0);
    // m_raiseMotor.setSoftLimit(SoftLimitDirection


    
   

    
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



    //set some soft limit to prevent full extension (illegal)
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    // checkIfStalled();
    s_extend.isSafeTemp();
    s_raise.isSafeTemp();
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

  


  

  

@Log.ToString(name = "Arm Pose")
public Translation2d getArmPose(){

double armangle = getArmAngle();

double armlength = k_minLength + s_extend.getPosition()/k_rangeLengthPos * (k_maxLength-k_minLength);

double clawhieght = k_pivotHeight+ Math.sin(Math.toRadians(armangle))*armlength; 
double clawlength = Math.cos(Math.toRadians(armangle))*armlength-k_columtToFront;




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

public double getArmLength(){
  double length = 0;
  //use law of cosines because we have three sides
  //c^2 = a^2+b^2-2ab*cos(C)
  //a^2+b^2 - c^2 = 2ab*cos(C)
  //C = acos((a^2+b^2 - c^2) /2ab)
  // double a = k_pivotLenght;
  // double b = k_pivotOffset;
  // double c = m_raiseEncoder.getPosition()/k_rangeLena*(k_maxLeangle-k_minLeangle)+k_minLeangle;
  double extent = s_extend.getPosition()/k_rangeLengthPos*(k_maxLength-k_minLength)+k_minLength;
  length = extent * Math.cos(getArmAngle())- k_columtToFront;
  return length;

}



public double angleToLeangle(double angle){
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
  double leangle = (c-k_minLeangle)/(k_maxLeangle-k_minLeangle)*k_maxLeanglePos;
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
  return s_extend.getPosition();
}

@Log(name = "Extent")
public double getExtent(){
  return s_extend.getPosition()/k_rangeLengthPos*(k_maxLength-k_minLength)+k_minLength;
}

@Log(name = "Raise Position")
public double getRaisedPosition(){
  return s_raise.getPosition();
}
@Log(name = "Grip Position")
public double getGripPosition(){
  return m_gripEncoder.getPosition();
}
 public void setArmPosition(Translation2d position){
    //takes in a pose and sets the arm to that position
    Translation2d setpoints = cartToLengths(position.getX(), position.getY());

    s_extend.setPosition(setpoints.getY(), 0);
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

public void setExtendMotion(double position){
  //inpoutrange -1 to 1
  position = s_extend.mapInput(position);

  vExtendSetPos= s_extend.setMotion(position, 0);
}
public void setExtendMotionOnboard(double position){
  position = s_extend.mapInput(position);
  vExtendSetPos=s_extend.setMotionOB(position,0);
}

public void setExtendPositionSafe(double position){
  position = s_extend.mapInput(position);
  // calculate max extension based on arm angle so it doesnt hit the floor
  //max extion == k_pivotHeight/Math.cos(angle)
  
  // final double k_horizontalPos = -50;//????? this is a guess, fill in with actual value
  // final double k_minAngle = -30;//  ????? this is a guess, fill in with actual value by mesuring the angle of the arm
  double angle = Math.toRadians( getArmAngle());
  double maxExtension = position;
  if (angle < 0){
     maxExtension = ( k_pivotHeight-k_minArmHeight ) / Math.cos(angle);
  }
  maxExtension = (maxExtension-k_minLength)/(k_maxLength-k_minLength)*(k_rangeLengthPos);

  if (position > maxExtension){
    position = maxExtension;
  }
  vExtendSetPos=position;
  vMaxExtention = maxExtension;


  vExtendSetPos =  s_extend.setPosition(position ,0);
}
public void setExtendMotionSafe(double position){
  //inpoutrange -1 to 1
  
  // calculate max extension based on arm angle so it doesnt hit the floor
  //max extion == k_pivotHeight/Math.cos(angle)
  
  // final double k_horizontalPos = -50;//????? this is a guess, fill in with actual value
  // final double k_minAngle = -30;//  ????? this is a guess, fill in with actual value by mesuring the angle of the arm

  position = s_extend.mapInput(position);
  double angle = getArmAngle();
  double maxExtension = position;
  if (angle < 0){
     maxExtension = ( k_pivotHeight-k_minArmHeight ) / Math.cos(angle);
  }
  maxExtension = (maxExtension-k_minLength)/(k_maxLength-k_minLength)*(k_rangeLengthPos);

  if (position > maxExtension){
    position = maxExtension;
  }
  
  vMaxExtention = maxExtension;
  vExtendSetPos=s_extend.setMotion(position, 0);
  
}


public void setRaisePosition(double position){
  //inpoutrange -1 to 1
  //pos 0 is fully raised
  //negative pos is goint down 
  position = s_raise.mapInput(position);
  vRaisePos=s_raise.setPosition(position, 0);

}
public void setRaiseMotion(double position){
  //inpoutrange -1 to 1
  //pos 0 is fully raised
  //negative pos is goint down 
 position = s_raise.mapInput(position);

  
  s_raise.setMotion(position, 0);
}
public void setRaisePositionSafe(double position){
  //inpoutrange -1 to 1
  //pos 0 is fully raised
  //negative pos is goint down 
  position = s_raise.mapInput(position);


  double lenght = getExtent();
 // min angle = Math.cos(angle)= k_pivotHeight/length
  double minAngle = -Math.acos((k_pivotHeight-k_minArmHeight)/lenght);
  minAngle = angleToLeangle(minAngle)*k_rangeLeanglePos;


  if (position < minAngle){
    position = minAngle;
  }
  vRaisePos=position;
  vMinAngle = minAngle;


  vRaisePos=s_raise.setPosition(position, 0);

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




public void setRaiseMotionSafe( double position){
  //inpoutrange -1 to 1
  //pos 0 is fully raised
  //negative pos is goint down 
  position = s_raise.mapInput(position);
  // calcuate min angle/leangle based on arm extension so it doesnt hit the floor
  
  double lenght = getExtent();
 // min angle = Math.cos(angle)= k_pivotHeight/length
  double minAngle =Math.toDegrees( -Math.acos((k_pivotHeight-k_minArmHeight)/lenght));
  double minLeangle = angleToLeangle(minAngle)*k_rangeLeanglePos;
  
  if (position < minLeangle){
    position = minLeangle;
  }
  vRaisePos=s_raise.setMotion(position, 0);
  vMinAngle = minAngle;

  

}
public void toggleGripCone(){
  if(m_gripEncoder.getPosition()<38)
    setClawPosition(.94);

  else 
    setClawPosition(-1);

}
public void toggleGripCube(){
  if(m_gripEncoder.getPosition()<5 || m_gripEncoder.getPosition()>30)
    setClawPosition(0.1);

  else 
    setClawPosition(-1);
}

public void resetPosition(){
  s_extend.setEncPosition(0);
  s_raise.setEncPosition(0);
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











@Config (name = "Grip Max Output", tabName = "Arm")
void setGripMaxOutput(@Config(defaultValueNumeric = kMaxOutputG) double gripPower, @Config(defaultValueNumeric = kMinOutputG) double releasePower){
  
  m_gripPID.setOutputRange(-gripPower, releasePower);
}

 

@Config.NumberSlider (name = "min height", tabName = "Arm", min = 0, max = 8, defaultValue = d_minArmHeight)
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