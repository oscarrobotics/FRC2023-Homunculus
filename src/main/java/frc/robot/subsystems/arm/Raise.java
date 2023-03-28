package frc.robot.subsystems.arm;

import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkMaxPIDController;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import io.github.oblarg.oblog.Loggable;
import io.github.oblarg.oblog.annotations.Config;
import io.github.oblarg.oblog.annotations.Log;
import com.revrobotics.CANSparkMax.SoftLimitDirection;
import com.revrobotics.CANSparkMax.IdleMode;

public class Raise implements Loggable {

  public final CANSparkMax m_raiseMotor;
RelativeEncoder m_Encoder ;
SparkMaxPIDController m_PID;

    public final double kP_up_in = 0.013;
    public final double kI_up_in = 0.000;
    public final double kD_up_in = 0.0000;
    public final double kIz_up_in = 4;
    public final double kFF_up_in = -0.1;

    public final double kP_down_in = 0.06;
    public final double kI_down_in = 0.0008;
    public final double kD_down_in = 0.0000;
    public final double kIz_down_in = 4;
    public final double kFF_down_in = 0.0;

    public final double kP_up_out = 0.06;
    public final double kI_up_out = 0.0008;
    public final double kD_up_out = 0.0000;
    public final double kIz_up_out = 4;
    public final double kFF_up_out = 0.0;

    public final double kP_down_out = 0.06;
    public final double kI_down_out = 0.0008;
    public final double kD_down_out = 0.0000;
    public final double kIz_down_out = 4;
    public final double kFF_down_out = 0.0;
    

  

    public final double kMaxOutput_in = 0.3; //arm up
    public final double kMinOutput_in = 0.2;//arm down

    public final double kMaxOutput_out = 0.4; //arm up
    public final double kMinOutput_out = 0.3;//arm down


    public final double maxRPM = 5;
    public final double maxAccel = 1.0;
    public final double allowedErr = 0.5;
    public final double closedRR  = 2;

    public int kSlotIdx_up_in = 0;
    public int kSlotIdx_down_in = 1;
    public int kSlotIdx_up_out = 2;
    public int kSlotIdx_down_out = 3;

    public double vSetPos = 0;


    public Raise() {
      m_raiseMotor = new CANSparkMax(21,MotorType.kBrushless);
      m_Encoder = m_raiseMotor.getEncoder();
      m_PID = m_raiseMotor.getPIDController();

      m_raiseMotor.setSoftLimit(SoftLimitDirection.kForward, (float) 0);
      m_raiseMotor.setSoftLimit(SoftLimitDirection.kReverse, (float)-Arm.k_rangeLeanglePos);

      m_raiseMotor.setIdleMode(IdleMode.kBrake);
      m_raiseMotor.setSmartCurrentLimit(20, 50, 0);
      m_raiseMotor.setClosedLoopRampRate(closedRR);


      initPID();

    }

    // public double setArbituaryFF(double kFF_arb){
      
    // }

    public double  mapInput(double position){
      //inpoutrange -1 to 1
        // pos zero is fully retracted
        //1 is fully Raiseed
        // armSmother.calculate(5);
        position = Math.min(position, 1);
        position = Math.max(position, -1);

        position = position +1;
        position = position/2;
        position = position * (Arm.k_rangeLeanglePos-1)*-1;
        position = position-1;
        vSetPos = position;
        return position;
      
      }
      
       public double setPosition(double position, int slot){
      
        
        m_PID.setReference(position, CANSparkMax.ControlType.kPosition,  slot);
        return position;
      }
      public double setPosition(double position, int slot, double feedforward){
      
        
        m_PID.setReference(position, CANSparkMax.ControlType.kPosition,  slot, feedforward);
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

      // @Log(name = "Set Position", tabName = "Raise" ,rowIndex = 1 , columnIndex = 5)
      public double getSetPosition(){
        return vSetPos;
      }
      
      public double getPosition(){
        return m_Encoder.getPosition();
      }
      public double setEncPosition(double position){
  
        m_Encoder.setPosition(position);
        return position;
      }
      
      //  @Log.Graph(name = "Motor Curent" ,tabName = "Raise" ,rowIndex = 0 , columnIndex = 6)
       public double getCurrent(){
         return m_raiseMotor.getOutputCurrent();
       }
       @Log(name = "Motor Temp" ,tabName = "Raise", rowIndex = 3 , columnIndex = 6)
        public double getTemp(){
          return m_raiseMotor.getMotorTemperature();
        }

        @Log.BooleanBox(name = "Is Motor Safe", tabName = "Raise", rowIndex =1 , columnIndex = 10)
        public boolean isSafeTemp(){
          if(m_raiseMotor.getMotorTemperature() > 65.0){
            m_raiseMotor.setSmartCurrentLimit(2,2,0);
            return false;
          }
          return true;
        }
        // @Log.Graph (name = "Error", tabName = "Raise" ,rowIndex = 3, columnIndex = 0)
         public double getError(){
           return m_Encoder.getPosition() - vSetPos;
         }

        // @Log.Graph (name = "Percent Error", tabName = "Raise", rowIndex = 3, columnIndex = 3)
         public double getPercentError(){
              return (m_Encoder.getPosition() - vSetPos)/Arm.k_rangeExtentPos*100;
         }

         @Config (name = "P_up_in", tabName = "Raise", defaultValueNumeric = kP_up_in, rowIndex = 0, columnIndex = 0)
        void setP_up_in(double p){
          m_PID.setP(p , kSlotIdx_up_in);
        }
        @Config (name = "I_up_in", tabName = "Raise", defaultValueNumeric = kI_up_in , rowIndex = 0, columnIndex = 1)
        void setI_up_in(double i){
          m_PID.setI(i, kSlotIdx_up_in);
        }
        @Config (name = "D_up_in", tabName = "Raise", defaultValueNumeric = kD_up_in, rowIndex = 0, columnIndex = 2)
        void setD_up_in(double d){
          m_PID.setD(d, kSlotIdx_up_in);
        }
        @Config (name = "Iz_up_in", tabName = "Raise", defaultValueNumeric = kIz_up_in, rowIndex = 0, columnIndex = 3)
        void setIz_up_in(double iz){
          m_PID.setIZone(iz, kSlotIdx_up_in);
        }
        @Config (name = "F_up_in", tabName = "Raise", defaultValueNumeric = kFF_up_in, rowIndex = 0, columnIndex = 4)
        void setF_up_in(double f){
          m_PID.setFF(f, kSlotIdx_up_in);
        }
        @Config (name = "P_down_in", tabName = "Raise", defaultValueNumeric = kP_down_in, rowIndex = 1, columnIndex = 0)  
        void setP_down_in(double p){
          m_PID.setP(p , kSlotIdx_down_in);
        }
        @Config (name = "I_down_in", tabName = "Raise", defaultValueNumeric = kI_down_in, rowIndex = 1, columnIndex = 1)   

        void setI_down_in(double i){
          m_PID.setI(i, kSlotIdx_down_in);
        }
        @Config (name = "D_down_in", tabName = "Raise", defaultValueNumeric = kD_down_in, rowIndex = 1, columnIndex = 2)
        void setD_down_in(double d){
          m_PID.setD(d, kSlotIdx_down_in);
        }
        @Config (name = "Iz_down_in", tabName = "Raise", defaultValueNumeric = kIz_down_in, rowIndex = 1, columnIndex = 3)
        void setIz_down_in(double iz){
          m_PID.setIZone(iz, kSlotIdx_down_in);
        }
        @Config (name = "F_down_in", tabName = "Raise", defaultValueNumeric = kFF_down_in, rowIndex = 1, columnIndex = 4)
        void setF_down_in(double f){
          m_PID.setFF(f, kSlotIdx_down_in);
        }
        // @Config (name = "P_up_out", tabName = "Raise", defaultValueNumeric = kP_up_out)
        // void setP_up_out(double p){
        //   m_PID.setP(p , kSlotIdx_up_out);
        // }
        // @Config (name = "I_up_out", tabName = "Raise", defaultValueNumeric = kI_up_out)
        // void setI_up_out(double i){
        //   m_PID.setI(i, kSlotIdx_up_out);
        // }
        // @Config (name = "D_up_out", tabName = "Raise", defaultValueNumeric = kD_up_out)
        // void setD_up_out(double d){
        //   m_PID.setD(d, kSlotIdx_up_out);
        // }
        // @Config (name = "Iz_up_out", tabName = "Raise", defaultValueNumeric = kIz_up_out)
        // void setIz_up_out(double iz){
        //   m_PID.setIZone(iz, kSlotIdx_up_out);
        // }
        // @Config (name = "F_up_out", tabName = "Raise", defaultValueNumeric = kFF_up_out)
        // void setF_up_out(double f){
        //   m_PID.setFF(f, kSlotIdx_up_out);
        // }
        // @Config (name = "P_down_out", tabName = "Raise", defaultValueNumeric = kP_down_out)

        // void setP_down_out(double p){
        //   m_PID.setP(p , kSlotIdx_down_out);
        // }
        // @Config (name = "I_down_out", tabName = "Raise", defaultValueNumeric = kI_down_out)
        // void setI_down_out(double i){
        //   m_PID.setI(i, kSlotIdx_down_out);
        // }
        // @Config (name = "D_down_out", tabName = "Raise", defaultValueNumeric = kD_down_out)
        // void setD_down_out(double d){
        //   m_PID.setD(d, kSlotIdx_down_out);
        // }
        // @Config (name = "Iz_down_out", tabName = "Raise", defaultValueNumeric = kIz_down_out)
        // void setIz_down_out(double iz){
        //   m_PID.setIZone(iz, kSlotIdx_down_out);
        // }
        // @Config (name = "F_down_out", tabName = "Raise", defaultValueNumeric = kFF_down_out)
        // void setF_down_out(double f){
        //   m_PID.setFF(f, kSlotIdx_down_out);
        // }
  



        @Config (name = "MaxOutput_in", tabName = "Raise"  ,rowIndex = 2, columnIndex = 0, defaultValueNumeric = kMaxOutput_in)
        void setMaxOutput_in(@Config(defaultValueNumeric = kMaxOutput_in) double upPower ,@Config(defaultValueNumeric = kMinOutput_in)  double downPower){
          m_PID.setOutputRange(-downPower, upPower, kSlotIdx_up_in);
          m_PID.setOutputRange(-downPower, upPower, kSlotIdx_down_in);

          
        }
        @Config (name = "MaxOutput_out", tabName = "Raise", defaultValueNumeric = kMaxOutput_out, rowIndex = 2, columnIndex = 1)
        void setMaxOutput_out( @Config(defaultValueNumeric = kMaxOutput_out) double upPower ,@Config(defaultValueNumeric = kMinOutput_out)  double downPower){
          m_PID.setOutputRange(-downPower, upPower, kSlotIdx_up_out);
          m_PID.setOutputRange(-downPower, upPower, kSlotIdx_down_out);
          
        }

        @Config (name = "ClosedRR", tabName = "Raise", defaultValueNumeric = closedRR , rowIndex = 1, columnIndex = 5)
        void setClosedRR(double rr){
          m_raiseMotor.setClosedLoopRampRate(rr);
        }
    
        @Config (name = "Raise Voltage", tabName = "Raise FF" , rowIndex = 0, columnIndex = 0)
        public double getVoltage(){
          return m_raiseMotor.getBusVoltage()*m_raiseMotor.getAppliedOutput();
        
        }
        //No need to worry aout max inpt (motor will always be fighting gravity)



     private void initPID(){


      m_PID.setP(kP_up_in, kSlotIdx_up_in);
      m_PID.setI(kI_up_in, kSlotIdx_up_in);
      m_PID.setD(kD_up_in, kSlotIdx_up_in);
      m_PID.setIZone(kIz_up_in, kSlotIdx_up_in);
      m_PID.setFF(kFF_up_in, kSlotIdx_up_in);
      m_PID.setOutputRange(kMinOutput_in, kMaxOutput_in, kSlotIdx_up_in);

      m_PID.setP(kP_down_in, kSlotIdx_down_in);
      m_PID.setI(kI_down_in, kSlotIdx_down_in);
      m_PID.setD(kD_down_in, kSlotIdx_down_in);
      m_PID.setIZone(kIz_down_in, kSlotIdx_down_in);
      m_PID.setFF(kFF_down_in, kSlotIdx_down_in);
      m_PID.setOutputRange(kMinOutput_in, kMaxOutput_in, kSlotIdx_down_in);

      m_PID.setP(kP_up_out, kSlotIdx_up_out);
      m_PID.setI(kI_up_out, kSlotIdx_up_out);
      m_PID.setD(kD_up_out, kSlotIdx_up_out);
      m_PID.setIZone(kIz_up_out, kSlotIdx_up_out);
      m_PID.setFF(kFF_up_out, kSlotIdx_up_out);
      m_PID.setOutputRange(kMinOutput_out, kMaxOutput_out, kSlotIdx_up_out);

      m_PID.setP(kP_down_out, kSlotIdx_down_out);
      m_PID.setI(kI_down_out, kSlotIdx_down_out);
      m_PID.setD(kD_down_out, kSlotIdx_down_out);
      m_PID.setIZone(kIz_down_out, kSlotIdx_down_out);
      m_PID.setFF(kFF_down_out, kSlotIdx_down_out);
      m_PID.setOutputRange(kMinOutput_out, kMaxOutput_out, kSlotIdx_down_out);

      m_PID.setSmartMotionMaxVelocity(maxRPM, kSlotIdx_up_in);
      m_PID.setSmartMotionMaxVelocity(maxRPM, kSlotIdx_down_in);
      m_PID.setSmartMotionMaxVelocity(maxRPM, kSlotIdx_up_out);
      m_PID.setSmartMotionMaxVelocity(maxRPM, kSlotIdx_down_out);

      m_PID.setSmartMotionMaxAccel(maxAccel, kSlotIdx_up_in);
      m_PID.setSmartMotionMaxAccel(maxAccel, kSlotIdx_down_in);
      m_PID.setSmartMotionMaxAccel(maxAccel, kSlotIdx_up_out);
      m_PID.setSmartMotionMaxAccel(maxAccel, kSlotIdx_down_out);

      m_PID.setSmartMotionAllowedClosedLoopError(allowedErr, kSlotIdx_up_in);
      m_PID.setSmartMotionAllowedClosedLoopError(allowedErr, kSlotIdx_down_in);
      m_PID.setSmartMotionAllowedClosedLoopError(allowedErr, kSlotIdx_up_out);
      m_PID.setSmartMotionAllowedClosedLoopError(allowedErr, kSlotIdx_down_out);




    }
    

}
