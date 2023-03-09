package frc.robot.subsystems.arm;

import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import io.github.oblarg.oblog.Loggable;
import io.github.oblarg.oblog.annotations.Log;
import com.revrobotics.AbsoluteEncoder;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import com.revrobotics.SparkMaxAbsoluteEncoder.Type;

public class Arm extends SubsystemBase implements Loggable{
  
public final CANSparkMax m_extendMotor;
public final CANSparkMax m_raiseMotor;
public final CANSparkMax m_gripMotor;

RelativeEncoder  m_extendEncoder ;
    RelativeEncoder  m_raiseEncoder ;
    RelativeEncoder m_gripEncoder;

private final double k_maxLengthPos = 0;//postion at the max length we mesures in units
private final double k_maxLeanglePos = 0 ; //max length of the angle control in units
private final double k_maxClaw = 0;

private final double k_maxHeight = 6.25 ;// hieght in feet that the robot is alowed to be
private final double k_pivotHeight = 22; // hieght in inches that the pivot point oi the arm is of the floor

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


private final double k_pivotLenght = 12 ;//? the hieght of the arm pivot from the base of the angle actuator fill in actual
private final double k_pivotOffset = 6; //distance between the arm pirvot point and the place where the angle motor attaches to the arm, 
private final double k_minLeangle= 0;//used for kinematics
private final double k_maxLeangle=0;//used for kinematic
private final double k_minLength=0;
private final double k_maxLengh=0;
private final double k_columtToFront = 0; //distance of the colloum from the front of the robot

private final double k_ticksPerInchExtend=1;
private final double k_ticksPerInchRaise=1;
private final double k_ticksPerInchGrip= 1;

// private final AbsoluteEncoder m_Encoder;

  public Arm(){
    m_extendMotor = new CANSparkMax(5, MotorType.kBrushless);
   m_raiseMotor = new CANSparkMax(0,MotorType.kBrushless);
   m_gripMotor = new CANSparkMax(0,MotorType.kBrushless) ;
   
   m_extendMotor.setSmartCurrentLimit(2, 2, 0);
   m_raiseMotor.setSmartCurrentLimit(2,2,0);
   m_gripMotor.setSmartCurrentLimit(2,2,0);
    
    
  
    m_extendEncoder = m_extendMotor.getEncoder();
    m_raiseEncoder = m_raiseMotor.getEncoder();
    m_gripEncoder = m_gripMotor.getEncoder();


    m_extendEncoder.setPositionConversionFactor(k_ticksPerInchExtend);
    m_raiseEncoder.setPositionConversionFactor(k_ticksPerInchRaise );
    m_gripEncoder.setPositionConversionFactor(k_ticksPerInchGrip);

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

@Log(name = "Arm Pose")
public Translation2d getArmPose(){

double armangle = getArmAngle();

double armlength = k_minLength + m_extendEncoder.getPosition();

double clawhieght = k_pivotHeight+ Math.sin(armangle)*armlength; 
double clawlength =Math.cos(armangle)*armlength-k_columtToFront;




    Translation2d armpos = new Translation2d(clawlength, clawhieght);
  return armpos;
}

public double getArmAngle(){
  double angle = 0;
  //use law of cosines because we have three sides
  //c^2 = a^2+b^2-2ab*cos(C)
  //a^2+b^2 - c^2 = 2ab*cos(C)
  //C = acos((a^2+b^2 - c^2) /2ab)
  double a = k_pivotLenght;
  double b = k_pivotOffset;
  double c = m_raiseEncoder.getPosition();
  angle = Math.acos((Math.pow(a, 2)+Math.pow(b, 2)-Math.pow(c, 2))/(2*a*b)); // angle of obtuse triangle formed by arm
  angle = angle -90;// angle of arm  above or below horizontal
  



  return angle;
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
  return c;


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

@Log.Graph(name = "Extent Position")
public double getExtendPosition(){
  return m_extendEncoder.getPosition();
}

@Log.Graph(name = "Raise Position")
public double getRaisePosition(){
  return m_raiseEncoder.getPosition();
}
@Log.Graph(name = "Grip Position")
public double getGripPosition(){
  return m_gripEncoder.getPosition();
}

}