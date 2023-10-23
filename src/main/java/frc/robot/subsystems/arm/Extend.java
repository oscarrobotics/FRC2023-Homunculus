package frc.robot.subsystems.arm;



import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkMaxPIDController;
import edu.wpi.first.hal.simulation.RoboRioDataJNI;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
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


import io.github.oblarg.oblog.Loggable;

public class Extend implements Loggable{

  public final CANSparkMax m_extendMotor;
  RelativeEncoder  m_Encoder ;
  SparkMaxPIDController m_PID;

  //extend
//  public final double kPE_pos = 0.04;//0.14
//  public final double kIE_pos = 0.00002;//was 0.05
//  public final double kDE_pos = 0.02;//was 0.01
//  public final double kIzE_pos = 6;
//  public final double kFFE_pos = 0.0;

 public final double kP_vel = 0.00;//0.14
 public final double kI_vel = 0.000;//was 0.05
 public final double kD_vel = 0.0;//was 0.01
 public final double kIz_vel = 6;
 public final double kFF_vel = 0.0001;

//  public final double kFF_arbdef = 1.084;
 //  public double kFF_arb = 0.784;
 public final double kFF_arbDefault = 1.004;//1.084S
  public double kFF_arb = 1.004;
  // public double kFF_arb = 0;
  public final double kFF_arbCDefault = 0.184;
   public double kFF_arbC = 0.184;

 public final double kPE_pos = 0.05;//0.14
 public final double kIE_pos = 0.00000;//was 0.05
 public final double kDE_pos = 0.000;//was 0.01
 public final double kIzE_pos = 6;
 public final double kFFE_pos = 0.0;


 public final double kMaxOutputE_out = 0.3; //arm out?
 public final double kMinOutputE_out = 0.3;//arm in?

 public final double kMaxOutputE_in = 0.3; //arm out?
 public final double kMinOutputE_in = 0.3;//arm in?

 public final double kMaxOutputE_low = 0.4; //arm out?
 public final double kMinOutputE_low = 0.3;//arm in?

 public final double maxRPM = 5;
 public final double maxAccel = 2;
 public final double allowedErr = 0.5;
 public final double closedRR = 0.018;

public double maxVelocity = 400;



 int kSlotIdxPos  = 0;
 int kSlotIdxVel = 1;


 private double vSetPos = 0;

 private double retractPos = 0;
 private Timer settlTimer = new Timer() ;

 public Extend(){
  m_extendMotor = new CANSparkMax(20, MotorType.kBrushless);
  m_extendMotor.setSmartCurrentLimit(15 , 40,0); 
 
  m_extendMotor.setSoftLimit(SoftLimitDirection.kForward, (float)Arm.k_rangeExtentPos);
  m_extendMotor.setSoftLimit(SoftLimitDirection.kReverse, (float)0);

  m_extendMotor.setIdleMode(IdleMode.kBrake);
  m_extendMotor.setClosedLoopRampRate(closedRR);
  // m_extendMotor.setSecondaryCurrentLimit(40);

  m_Encoder = m_extendMotor.getEncoder();
  m_Encoder.setPositionConversionFactor(Arm.k_ticksPerInchExtend);
  m_PID = m_extendMotor.getPIDController();
  m_PID.setSmartMotionAllowedClosedLoopError(10, kSlotIdxVel);
  m_PID.setSmartMotionAllowedClosedLoopError(10, kSlotIdxPos);

  initPID();
 }








public double  mapInput(double position){
//inpoutrange -1 to 1
  // pos zero is fully retracted
  //1 is fully extended
  // armSmother.calculate(5)
  //limit the domain to -1 to 1
  position = Math.min(position, 1);
  position = Math.max(position, -1);

  position = position +1;
  position = position/2;
  position = position * (Arm.k_rangeExtentPos-1);
  position = position+1;
  
  return position;

}


 public double setPosition(double position, int slot){





  vSetPos = position;
  
  m_PID.setReference(position, CANSparkMax.ControlType.kPosition, slot);
  // m_PID.setReference(position-2, CANSparkMax.ControlType.kPosition, slot);
  return position;
}
public double setPosition(double position, int slot, double feedforward, boolean isStowed){
  // if (Math.abs(position-vSetPos)<3 && settlTimer.hasElapsed(2)){
  //   position = m_Encoder.getPosition();
      
  // }
  // if( Math.abs(position-vSetPos)>=3 ){
  //   settlTimer.restart();
  // }



  vSetPos = position;

  
  /*Added Stowing algorithm --> if the arm is stowed, set the voltage to 0 (since we're not using the arm in that case, and providing the
   * unextended arm extra voltage would be unnecessary.)
   * 
   * Keep note of DeMorgen's Law for boolean operators
   * 
   * Also make sure to fix limits
   */
  if(!isStowed || (position > 4 || getPosition() > 4)){
  m_PID.setReference(position, CANSparkMax.ControlType.kPosition, slot, feedforward);
  }
  else{
    m_PID.setReference(0, ControlType.kVoltage);
  }
  return position;
}

public double setVelocity(double velocityRatio,double arbFF){
  double velocity = velocityRatio * maxVelocity;
  m_PID.setReference(velocity, CANSparkMax.ControlType.kVelocity, 1,arbFF);
  return velocity;
}



public double setMotion(double position , int slot){
  
  m_PID.setReference(position, CANSparkMax.ControlType.kSmartMotion, slot);
  return position;
}


public double setMotionOB(double position , int slot){
  
  m_PID.setReference(position, CANSparkMax.ControlType.kSmartMotion, slot);
  return position;
}


//arbff helper functions
public double maxVoltage=0;
public double maxVoltagesStoped=0;
public void setVoltage(double dutycylce){
  double voltage = dutycylce * 3;

  m_extendMotor.setVoltage(voltage);
}
// @Log(name = "Voltage", tabName = "Extend FF", rowIndex = 0, columnIndex = 0)
public double getVoltage(){
  return m_extendMotor.getBusVoltage()*m_extendMotor.getAppliedOutput();

} 
// @Log(name = "Max Voltage", tabName = "Extend FF", rowIndex = 0, columnIndex = 1)
public double getMaxVoltage(){
  if (Math.abs(m_extendMotor.getBusVoltage()*m_extendMotor.getAppliedOutput() )> Math.abs(maxVoltage)){
    maxVoltage = m_extendMotor.getBusVoltage()*m_extendMotor.getAppliedOutput();
  }
  return maxVoltage;
}
// @Log(name = "Max Voltage Stopped", tabName = "Extend FF", rowIndex = 0, columnIndex = 2)
public double getMaxVoltageStopped(){
  if (Math.abs(m_extendMotor.getBusVoltage()*m_extendMotor.getAppliedOutput() )> Math.abs(maxVoltagesStoped)
  && (Math.abs(m_Encoder.getVelocity()) < Math.abs(0.3))){
    maxVoltagesStoped = m_extendMotor.getBusVoltage()*m_extendMotor.getAppliedOutput();
  }
  return maxVoltagesStoped;
}
//rested max voltages from oblog
// @Config.Command(name = "Reset Max Voltage", tabName = "Extend FF" , rowIndex = 0, columnIndex = 3)
public final Command resetMaxVoltage = new InstantCommand(this::resetMaxVoltage); 




public void resetMaxVoltage(){
  maxVoltage = 0;
  maxVoltagesStoped = 0;
}


//oblog methods

// @Log(name = "Extend Position", tabName = "Raise FF")
public double getPosition(){
  return m_Encoder.getPosition();

}
public double setEncPosition(double position){
  
  m_Encoder.setPosition(position);
  return position;
}

//  @Log.Graph(name = "Motor Curent" ,tabName = "Extend", rowIndex =0, columnIndex = 6 )
 public double getCurrent(){
   return m_extendMotor.getOutputCurrent();
 }
 @Log(name = "RMtr Temp" ,tabName = "TargetSelector", rowIndex = 4 , columnIndex = 1)
 @Log(name = "Motor Temp", tabName = "Extend", rowIndex =1 , columnIndex = 9)
  public double getTemp(){
    return m_extendMotor.getMotorTemperature();
  }

  @Log.BooleanBox(name = "Is Motor Safe", tabName = "Extend", rowIndex =1 , columnIndex = 10)
  public boolean isSafeTemp(){
    if(m_extendMotor.getMotorTemperature() > 65.0){
      m_extendMotor.setSmartCurrentLimit(2,2,2);
      return false;
    }
    return true;
  }
  
  // @Log.Graph (name = "Error", tabName = "Extend", rowIndex =2, columnIndex = 0)
  public double getError(){
    return m_Encoder.getPosition() - vSetPos;
  }
  // @Log.Graph (name = "Percent Error", tabName = "Extend", rowIndex =2, columnIndex = 3)
  public double getPercentError(){
    return (m_Encoder.getPosition() - vSetPos)/Arm.k_rangeExtentPos*100;
  }
  @Log.Graph (name = "Velocity", tabName = "Extend", rowIndex = 3, columnIndex = 6)
  public double getVelocity(){
    return m_Encoder.getVelocity();
  }
  @Log.Graph (name = "Velocity Error", tabName = "Extend", rowIndex = 3, columnIndex = 9)
  public double getVelocityError(){
    return (m_Encoder.getVelocity() - vSetPos)/maxVelocity*100;
  }



//PID setters
// @Config (name = "Extend P_out", tabName = "Extend", defaultValueNumeric = kPE_pos, rowIndex = 0, columnIndex = 0)
void setP_out(double p){
  m_PID.setP(p, kSlotIdxPos );
}
// @Config (name = "Extend I_out", tabName = "Extend", defaultValueNumeric = kIE_pos, rowIndex = 0, columnIndex = 1)
void setI_out(double i){
  m_PID.setI(i, kSlotIdxPos );
}
// @Config (name = "Extend D_out", tabName = "Extend", defaultValueNumeric = kDE_pos, rowIndex = 0, columnIndex = 2)
void setD_out(double d){
  m_PID.setD(d, kSlotIdxPos );
}
// @Config (name = "Extend Iz_out", tabName = "Extend", defaultValueNumeric = kIzE_pos, rowIndex = 0, columnIndex = 3)
void setIz_out(double iz){
  m_PID.setIZone(iz, kSlotIdxPos );
}
// @Config (name = "Extend FF_out", tabName = "Extend", defaultValueNumeric = kFFE_pos, rowIndex = 0, columnIndex = 4)
void setFF_out(double f){
  m_PID.setFF(f, kSlotIdxPos );
}
@Config (name = "Extend P_VEL", tabName = "Extend", defaultValueNumeric = kP_vel, rowIndex = 1,columnIndex = 0)
void setP_in(double p){
  m_PID.setP(p, kSlotIdxVel);
}
@Config (name = "Extend I_vel", tabName = "Extend", defaultValueNumeric = kI_vel ,  rowIndex = 1, columnIndex = 1)
void setI_in(double i){
  m_PID.setI(i, kSlotIdxVel);
}
@Config (name = "Extend D_vel", tabName = "Extend", defaultValueNumeric = kD_vel, rowIndex =1, columnIndex = 2 )
void setD_in(double d){
  m_PID.setD(d, kSlotIdxVel);
}
@Config (name = "Extend Iz_vel", tabName = "Extend", defaultValueNumeric = kIz_vel, rowIndex = 1, columnIndex = 3)
void setIz_in(double iz){
  m_PID.setIZone(iz, kSlotIdxVel);
}
@Config (name = "Extend FF_vel", tabName = "Extend", defaultValueNumeric = kFF_vel, rowIndex = 1, columnIndex = 4)
void setFF_in(double f){
  m_PID.setFF(f, kSlotIdxVel);
}
@Config (name = "Extend ff_arb", tabName = "Extend", defaultValueNumeric = kFF_arbDefault, rowIndex = 1, columnIndex = 5)
void setArbFF(double f){
  kFF_arb = f;
} 
@Config (name = "Extend ff_arbC", tabName = "Extend", defaultValueNumeric = kFF_arbCDefault, rowIndex = 1, columnIndex = 6)
void setArbFFC(double f){
  kFF_arbC = f;
}
// @Config (name = "Extend FF_arb", tabName = "Extend", defaultValueNumeric = kFF_arbdef, rowIndex = 1, columnIndex = 4)
// void setFF_arb(double FF_arb){
//   kFF_arb = FF_arb;
// }

// @Config (name = "Extend P_low", tabName = "Extend", defaultValueNumeric = kP_low)
// void setP_low(double p){
//   m_PID.setP(p, kSlotIdxLow);
// }
// @Config (name = "Extend I_low", tabName = "Extend", defaultValueNumeric = kI_low)
// void setI_low(double i){
//   m_PID.setI(i, kSlotIdxLow);
// }
// @Config (name = "Extend D_low", tabName = "Extend", defaultValueNumeric = kD_low)
// void setD_low(double d){
//   m_PID.setD(d, kSlotIdxLow);
// }
// @Config (name = "Extend Iz_low", tabName = "Extend", defaultValueNumeric = kIz_low)
// void setIz_low(double iz){
//   m_PID.setIZone(iz, kSlotIdxLow);
// }
// @Config (name = "Extend FF_low", tabName = "Extend", defaultValueNumeric = kFF_low)
// void setFF_low(double f){
//   m_PID.setFF(f, kSlotIdxLow);
// }
// @Config (name = "Max Output out", tabName = "Extend", rowIndex = 1, columnIndex = 0)
void setMaxOutputOut(@Config(defaultValueNumeric = kMaxOutputE_out) double outPower, @Config(defaultValueNumeric = kMinOutputE_out) double inPower){
  
  m_PID.setOutputRange(-inPower, outPower, kSlotIdxPos);

}
// @Config (name = "Max Output in", tabName = "Extend")
// void setMaxOutputIn(@Config(defaultValueNumeric = kMaxOutputE_in) double outPower, @Config(defaultValueNumeric = kMinOutputE_in) double inPower){
  
//   m_PID.setOutputRange(-inPower, outPower, kSlotIdxIn);
    


// }
// @Config (name = "Max Output low", tabName = "Extend")
// void setMaxOutputLow(@Config(defaultValueNumeric = kMaxOutputE_low) double outPower, @Config(defaultValueNumeric = kMinOutputE_low) double inPower){
  
//   m_PID.setOutputRange(-inPower, outPower, kSlotIdxLow);
// }
// @Config(name = "Cl RR", tabName = "Extend", rowIndex = 1, columnIndex = 5, defaultValueNumeric = closedRR)
void setClosedLoopRampRate(double rampRate){
  m_extendMotor.setClosedLoopRampRate(rampRate);
}


 private void initPID(){


  m_PID.setP(kPE_pos,kSlotIdxPos);
  m_PID.setI(kIE_pos,kSlotIdxPos);
  m_PID.setD(kDE_pos,kSlotIdxPos);
  m_PID.setIZone(kIzE_pos,kSlotIdxPos);
  m_PID.setFF(kFFE_pos,kSlotIdxPos);

  m_PID.setP(kP_vel,kSlotIdxVel);
  m_PID.setI(kI_vel,kSlotIdxVel);
  m_PID.setD(kI_vel, kSlotIdxVel);
  m_PID.setIZone(kIz_vel,kSlotIdxVel);
  m_PID.setFF(kFF_vel,kSlotIdxVel);



  m_PID.setOutputRange(-kMinOutputE_out, kMaxOutputE_out,kSlotIdxPos);
  m_PID.setOutputRange(-kMinOutputE_in, kMaxOutputE_in,kSlotIdxVel);

  
  m_PID.setSmartMotionMaxVelocity(maxRPM, kSlotIdxPos);
  m_PID.setSmartMotionMinOutputVelocity(-maxRPM, kSlotIdxPos);
  m_PID.setSmartMotionMaxAccel(maxAccel, kSlotIdxPos);
  m_PID.setSmartMotionAllowedClosedLoopError(10, kSlotIdxPos);

  m_PID.setSmartMotionMaxVelocity(maxRPM, kSlotIdxVel);
  m_PID.setSmartMotionMinOutputVelocity(-maxRPM, kSlotIdxVel);
  m_PID.setSmartMotionMaxAccel(maxAccel, kSlotIdxVel);
  m_PID.setSmartMotionAllowedClosedLoopError(10, kSlotIdxVel);

 
  m_PID.setSmartMotionAllowedClosedLoopError(allowedErr, kSlotIdxVel);
  m_PID.setSmartMotionAllowedClosedLoopError(allowedErr, kSlotIdxPos);


 }



  
}
