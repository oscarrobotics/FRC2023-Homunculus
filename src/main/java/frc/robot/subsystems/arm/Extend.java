package frc.robot.subsystems.arm;



import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkMaxPIDController;
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
 public final double kPE_out = 0.04;//0.14
 public final double kIE_out = 0.0002;//was 0.05
 public final double kDE_out = 0.01;//was 0.01
 public final double kIzE_out = 6;
 public final double kFFE_out = 0.0;

 public final double kP_in = 0.04;//0.14
 public final double kI_in = 0.000;//was 0.05
 public final double kD_in = 0.01;//was 0.01
 public final double kIz_in = 6;
 public final double kFF_in = 0.0;


 public final double kP_low = 0.06;//0.14
 public final double kI_low = 0.0008;//was 0.05
 public final double kD_low = 0.0000;//was 0.01
 public final double kIz_low = 4;
 public final double kFF_low = 0.0;

 public final double kMaxOutputE_out = 0.3; //arm out?
 public final double kMinOutputE_out = 0.3;//arm in?

 public final double kMaxOutputE_in = 0.3; //arm out?
 public final double kMinOutputE_in = 0.3;//arm in?

 public final double kMaxOutputE_low = 0.4; //arm out?
 public final double kMinOutputE_low = 0.3;//arm in?

 public final double maxRPM = 5;
 public final double maxAccel = 2;
 public final double allowedErr = 0.5;
 public final double closedRR = 0.3;

 int kSlotIdxOut  = 0;
 int kSlotIdxIn = 1;
 int kSlotIdxLow =2;

 private double vSetPos = 0;

 public Extend(){
  m_extendMotor = new CANSparkMax(20, MotorType.kBrushless);
  m_extendMotor.setSmartCurrentLimit(15 , 40,0); 
 
  m_extendMotor.setSoftLimit(SoftLimitDirection.kForward, (float)Arm.k_rangeLengthPos);
  m_extendMotor.setSoftLimit(SoftLimitDirection.kReverse, (float)0);

  m_extendMotor.setIdleMode(IdleMode.kBrake);
  m_extendMotor.setClosedLoopRampRate(closedRR);
  // m_extendMotor.setSecondaryCurrentLimit(40);

  m_Encoder = m_extendMotor.getEncoder();
  m_Encoder.setPositionConversionFactor(Arm.k_ticksPerInchExtend);
  m_PID = m_extendMotor.getPIDController();
  m_PID.setSmartMotionAllowedClosedLoopError(10, kSlotIdxIn);
  m_PID.setSmartMotionAllowedClosedLoopError(10, kSlotIdxOut);

  initPID();
 }








public double  mapInput(double position){
//inpoutrange -1 to 1
  // pos zero is fully retracted
  //1 is fully extended
  // armSmother.calculate(5);
  position = position +1;
  position = position/2;
  position = position * (Arm.k_rangeLengthPos-1);
  position = position+1;
  vSetPos = position;
  return position;

}


 public double setPosition(double position, int slot){
  
  
  m_PID.setReference(position, CANSparkMax.ControlType.kPosition, slot);
  // m_PID.setReference(position-2, CANSparkMax.ControlType.kPosition, slot);
  return position;
}
public double setPosition(double position, int slot, double feedforward){
 

  m_PID.setReference(position, CANSparkMax.ControlType.kPosition, slot, feedforward);
  return position;
}

public double setMotion(double position , int slot){
  
  m_PID.setReference(position, CANSparkMax.ControlType.kSmartMotion, slot);
  return position;
}

public double setMotionOB(double position , int slot){
  
  m_PID.setReference(position, CANSparkMax.ControlType.kSmartMotion, slot);
  return position;
}



@Log(name = "Extend Position", tabName = "Extend")
public double getPosition(){
  return m_Encoder.getPosition();

}
public double setEncPosition(double position){
  
  m_Encoder.setPosition(position);
  return position;
}

 @Log.Graph(name = "Motor Curent" ,tabName = "Extend", rowIndex =0, columnIndex = 6 )
 public double getCurrent(){
   return m_extendMotor.getOutputCurrent();
 }

 @Log(name = "Motor Temp", tabName = "Extend", rowIndex =1 , columnIndex = 9)
  public double getTemp(){
    return m_extendMotor.getMotorTemperature();
  }

  @Log.BooleanBox(name = "Is Motor Safe", tabName = "Extend", rowIndex =1 , columnIndex = 10)
  public boolean isSafeTemp(){
    if(m_extendMotor.getMotorTemperature() > 65.0){
      m_extendMotor.setVoltage(0);
      return false;
    }
    return true;
  }
  
  @Log.Graph (name = "Error", tabName = "Extend", rowIndex =2, columnIndex = 0)
  public double getError(){
    return m_Encoder.getPosition() - vSetPos;
  }
  @Log.Graph (name = "Percent Error", tabName = "Extend", rowIndex =2, columnIndex = 3)
  public double getPercentError(){
    return (m_Encoder.getPosition() - vSetPos)/Arm.k_rangeLengthPos*100;
  }



//PID setters
@Config (name = "Extend P_out", tabName = "Extend", defaultValueNumeric = kPE_out, rowIndex = 0, columnIndex = 0)
void setP_out(double p){
  m_PID.setP(p, kSlotIdxOut );
}
@Config (name = "Extend I_out", tabName = "Extend", defaultValueNumeric = kIE_out, rowIndex = 0, columnIndex = 1)
void setI_out(double i){
  m_PID.setI(i, kSlotIdxOut );
}
@Config (name = "Extend D_out", tabName = "Extend", defaultValueNumeric = kDE_out, rowIndex = 0, columnIndex = 2)
void setD_out(double d){
  m_PID.setD(d, kSlotIdxOut );
}
@Config (name = "Extend Iz_out", tabName = "Extend", defaultValueNumeric = kIzE_out, rowIndex = 0, columnIndex = 3)
void setIz_out(double iz){
  m_PID.setIZone(iz, kSlotIdxOut );
}
@Config (name = "Extend FF_out", tabName = "Extend", defaultValueNumeric = kFFE_out, rowIndex = 0, columnIndex = 4)
void setFF_out(double f){
  m_PID.setFF(f, kSlotIdxOut );
}
@Config (name = "Extend P_in", tabName = "Extend", defaultValueNumeric = kP_in, rowIndex = 1,columnIndex = 0)
void setP_in(double p){
  m_PID.setP(p, kSlotIdxIn);
}
@Config (name = "Extend I_in", tabName = "Extend", defaultValueNumeric = kI_in ,  rowIndex = 1, columnIndex = 1)
void setI_in(double i){
  m_PID.setI(i, kSlotIdxIn);
}
@Config (name = "Extend D_in", tabName = "Extend", defaultValueNumeric = kD_in, rowIndex =1, columnIndex = 2 )
void setD_in(double d){
  m_PID.setD(d, kSlotIdxIn);
}
@Config (name = "Extend Iz_in", tabName = "Extend", defaultValueNumeric = kIz_in, rowIndex = 1, columnIndex = 3)
void setIz_in(double iz){
  m_PID.setIZone(iz, kSlotIdxIn);
}
@Config (name = "Extend FF_in", tabName = "Extend", defaultValueNumeric = kFF_in, rowIndex = 1, columnIndex = 4)
void setFF_in(double f){
  m_PID.setFF(f, kSlotIdxIn);
}
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
@Config (name = "Max Output out", tabName = "Extend", rowIndex = 1, columnIndex = 0)
void setMaxOutputOut(@Config(defaultValueNumeric = kMaxOutputE_out) double outPower, @Config(defaultValueNumeric = kMinOutputE_out) double inPower){
  
  m_PID.setOutputRange(-inPower, outPower, kSlotIdxOut);

}
// @Config (name = "Max Output in", tabName = "Extend")
// void setMaxOutputIn(@Config(defaultValueNumeric = kMaxOutputE_in) double outPower, @Config(defaultValueNumeric = kMinOutputE_in) double inPower){
  
//   m_PID.setOutputRange(-inPower, outPower, kSlotIdxIn);
    


// }
// @Config (name = "Max Output low", tabName = "Extend")
// void setMaxOutputLow(@Config(defaultValueNumeric = kMaxOutputE_low) double outPower, @Config(defaultValueNumeric = kMinOutputE_low) double inPower){
  
//   m_PID.setOutputRange(-inPower, outPower, kSlotIdxLow);
// }
@Config(name = "Cl RR", tabName = "Extend", rowIndex = 1, columnIndex = 5, defaultValueNumeric = closedRR)
void setClosedLoopRampRate(double rampRate){
  m_extendMotor.setClosedLoopRampRate(rampRate);
}


 private void initPID(){


  m_PID.setP(kPE_out,kSlotIdxOut);
  m_PID.setI(kIE_out,kSlotIdxOut);
  m_PID.setD(kDE_out,kSlotIdxOut);
  m_PID.setIZone(kIzE_out,kSlotIdxOut);
  m_PID.setFF(kFFE_out,kSlotIdxOut);

  m_PID.setP(kP_in,kSlotIdxIn);
  m_PID.setI(kI_in,kSlotIdxIn);
  m_PID.setD(kI_in, kSlotIdxIn);
  m_PID.setIZone(kIz_in,kSlotIdxIn);
  m_PID.setFF(kFF_in,kSlotIdxIn);

  m_PID.setP(kP_low,kSlotIdxLow);
  m_PID.setI(kI_low,kSlotIdxLow);
  m_PID.setD(kI_low, kSlotIdxLow);
  m_PID.setIZone(kIz_low,kSlotIdxLow);
  m_PID.setFF(kFF_low,kSlotIdxLow);

  m_PID.setOutputRange(-kMinOutputE_out, kMaxOutputE_out,kSlotIdxOut);
  m_PID.setOutputRange(-kMinOutputE_in, kMaxOutputE_in,kSlotIdxIn);
  m_PID.setOutputRange(-kMinOutputE_low, kMaxOutputE_low,kSlotIdxLow);
  
  m_PID.setSmartMotionMaxVelocity(maxRPM, kSlotIdxOut);
  m_PID.setSmartMotionMinOutputVelocity(-maxRPM, kSlotIdxOut);
  m_PID.setSmartMotionMaxAccel(maxAccel, kSlotIdxOut);
  m_PID.setSmartMotionAllowedClosedLoopError(10, kSlotIdxOut);

  m_PID.setSmartMotionMaxVelocity(maxRPM, kSlotIdxIn);
  m_PID.setSmartMotionMinOutputVelocity(-maxRPM, kSlotIdxIn);
  m_PID.setSmartMotionMaxAccel(maxAccel, kSlotIdxIn);
  m_PID.setSmartMotionAllowedClosedLoopError(10, kSlotIdxIn);

  m_PID.setSmartMotionMaxVelocity(maxRPM, kSlotIdxLow);
  m_PID.setSmartMotionMinOutputVelocity(-maxRPM, kSlotIdxLow);
  m_PID.setSmartMotionMaxAccel(maxAccel, kSlotIdxLow);
  m_PID.setSmartMotionAllowedClosedLoopError(10, kSlotIdxLow);

  m_PID.setSmartMotionAllowedClosedLoopError(allowedErr, kSlotIdxLow);
  m_PID.setSmartMotionAllowedClosedLoopError(allowedErr, kSlotIdxIn);
  m_PID.setSmartMotionAllowedClosedLoopError(allowedErr, kSlotIdxOut);


 }
  
}
