package frc.robot.subsystems.arm;

import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.networktables.GenericEntry;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.ControllerButtons;
import frc.robot.RobotContainer;
import io.github.oblarg.oblog.Loggable;
import io.github.oblarg.oblog.annotations.Log;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;

import java.math.RoundingMode;

import com.revrobotics.AbsoluteEncoder;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.CANSparkMax.ControlType;
import com.revrobotics.CANSparkMax.IdleMode;
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


private final double k_maxLengthPos = -1.09;//postion at the max length we mesures in units
private final double k_minLengthPos = -40;//postion at the max length we mesures in units



private final double k_maxLeanglePos = 84.3 ; //max length of the angle control in units
private final double k_minLeanglePos = 0 ; //max length of the angle control in units
private final double k_maxClaw = 42.14;
private final double k_minClaw = -2.02;

// private final double k_rangeLengthPos = k_maxLengthPos - k_minLengthPos;
// private final double k_rangeLeanglePos = k_maxLeanglePos- k_minLeanglePos;
public final double k_buttonMinRange = -1;
public final double k_buttonMaxRange = 1;
private final double k_rangeLengthPos = 38.5;
private final double k_rangeMinPos = 0.0;
private final double k_rangeLeanglePos =120;
private final double k_rangeClaw = k_maxClaw - k_minClaw;


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
private GenericEntry m_kP;

public final double k_targetExtPosHigh = 0.0;
public final double k_targetRaisePosHigh = 0.0;
public double k_targetGripPosHigh;

// private final AbsoluteEncoder m_Encoder;
  public Arm(){
  
    m_kP =
      Shuffleboard.getTab("Arm")
          .add("Arm P", 0.05)
          .withWidget("Number Slider")
          .withPosition(1, 1)
          .withSize(2, 1)
          .getEntry();

   m_extendMotor = new CANSparkMax(20, MotorType.kBrushless);
   m_raiseMotor = new CANSparkMax(21,MotorType.kBrushless);
   m_gripMotor = new CANSparkMax(22,MotorType.kBrushless) ;
   
   m_gripMotor.setInverted(true);
   m_raiseMotor.setInverted(true);
   //TODO: manage if too snappy; be super careful with limits however
   m_extendMotor.setSmartCurrentLimit(5, 5, 0); 
   m_raiseMotor.setSmartCurrentLimit(5,5,0);
   m_gripMotor.setSmartCurrentLimit(5,5,0);

   m_extendMotor.setIdleMode(IdleMode.kBrake);
   m_raiseMotor.setIdleMode(IdleMode.kBrake);
   m_gripMotor.setIdleMode(IdleMode.kBrake);
    
   

    
  //encoder
    m_extendEncoder = m_extendMotor.getEncoder();
    m_raiseEncoder = m_raiseMotor.getEncoder();
    m_gripEncoder = m_gripMotor.getEncoder();


    m_extendEncoder.setPositionConversionFactor(k_ticksPerInchExtend);
    m_raiseEncoder.setPositionConversionFactor(k_ticksPerInchRaise );
    m_gripEncoder.setPositionConversionFactor(k_ticksPerInchGrip);



    m_extendPID = m_extendMotor.getPIDController();
    m_raisePID = m_raiseMotor.getPIDController();
    m_gripPID = m_gripMotor.getPIDController();

    m_extendPID.setP(5.0);
    m_extendPID.setI(0.00);
    m_extendPID.setD(0.01);
    m_extendPID.setIZone(2);
    m_extendPID.setFF(0);
    m_extendPID.setOutputRange(-0.3, 0.8);

    m_raisePID.setP(0.05);
    m_raisePID.setI(0.0005);
    m_raisePID.setD(0.001);
    m_raisePID.setIZone(2);
    m_raisePID.setFF(0);
    m_raisePID.setOutputRange(-0.3, 0.8);

    m_gripPID.setP(1);
    m_gripPID.setI(0);
    m_gripPID.setD(0);
    m_gripPID.setIZone(0);
    m_gripPID.setFF(0);
    m_gripPID.setOutputRange(-0.4, 0.4);


    //set some soft limit to prevent full extension (illegal)
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    // checkIfStalled();
    m_raisePID.setP(m_kP.getDouble(0.05)); 
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
  angle = angle -90;// i of arm  above or below horizontal
  



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

@Log(name = "Extent Position")
public double getExtendedPosition(){
  return m_extendEncoder.getPosition();
}

@Log(name = "Raise Position")
public double getRaisedPosition(){
  return m_raiseEncoder.getPosition();
}
@Log(name = "Grip Position")
public double getGripPosition(){
  return m_gripEncoder.getPosition();
}

public void resetPosition(){
  m_extendEncoder.setPosition(0);
  m_raiseEncoder.setPosition(0);
  m_gripEncoder.setPosition(0);
}
// public Command dropCargo(Translation2d position){
//   return new SequentialCommandGroup(
//     new InstantCommand(()->setClawPosition(0)),
//     new InstantCommand(()->setArmPosition(position)),
//     new WaitCommand(0.5),
//     new InstantCommand(()->setClawPosition(1)),
//     new WaitCommand(0.5),
//     new InstantCommand(()->setClawPosition(0)),
//     new InstantCommand(()->setArmPosition(new Translation2d(0,0)))
//   );

public void setExtentPosition(double position){
  //inpoutrange -1 to 1
  // pos zero is fully retracted
  //1 is fully extended
  //Check math, rangeconverter class

  //using linear conversion from button values -> CANSparkMax ticks
  position = ((position - k_buttonMinRange) /
        (k_buttonMaxRange - k_buttonMinRange)) * 
        (k_rangeLengthPos - 0) + 0;
  // position = position +1;
  // position = position/2;
  // position = position * (k_rangeLengthPos-2);
  // position = position+2;

  m_extendPID.setReference(position, CANSparkMax.ControlType.kPosition);
  SmartDashboard.putNumber("targetPos", position);
}
public void setRaisedPosition(double position){
  //inpoutrange -1 to 1
  //pos 0 is fully raised
  //negative pos is goint down 
  // position = ((getRaisedPosition() - k_buttonMinRange) /
  // (k_buttonMaxRange - k_buttonMinRange)) * 
  // (k_rangeLeanglePos - 0) + 0;
  // position = position +1;
  // position = position/2;
  // position = position * (k_rangeLeanglePos-2) *-1;
  // position = position -2;
  position = ((position - k_buttonMinRange) /
  (k_buttonMaxRange - k_buttonMinRange)) * 
  (k_rangeLeanglePos - 0) + 0;

  m_raisePID.setReference(position, CANSparkMax.ControlType.kPosition);


}
public void setClawPosition(double position){
  //inpoutrange -1 to 1
  //pos 0 is fully raised
  //negative pos is goint down 
  // position = position +1;
  // position = position/2;
  // position = position * k_rangeClaw ;
 position =  (RobotContainer.m_operator.arcadeWhiteLeft().getAsBoolean() ? -10 : 40);

 m_gripPID.setReference(position, CANSparkMax.ControlType.kPosition);
}
}