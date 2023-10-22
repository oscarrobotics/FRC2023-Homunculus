package frc.robot.subsystems.arm;

import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkMaxPIDController;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.math.filter.SlewRateLimiter;
import io.github.oblarg.oblog.Loggable;
import io.github.oblarg.oblog.annotations.Config;
import io.github.oblarg.oblog.annotations.Log;
import com.revrobotics.CANSparkMax.SoftLimitDirection;
import com.revrobotics.CANSparkMax.IdleMode;

public class Raise implements Loggable {

  private static final double kFF_arbADefault = -0.01;//angle based
  private static final double kFF_arbCDefault = 0.04;//extent based0.4
  private static final double kFF_arbeDefault = 0.012; //contstant 0.012
  public final CANSparkMax m_raiseMotor;
RelativeEncoder m_Encoder ;
SparkMaxPIDController m_PID;

    public final double kP_Pos = 0.06;//0.043
    public final double kI_Pos = 0.000;
    public final double kD_Pos = 0.0000;
    public final double kIz_Pos = 4;
    public final double kFF_Pos = 0;
  //position based feed forward
    public  double karb_ffa = -0.01; //angle based
    public double karb_ffe = 0.012; //extent based
    public double karb_ffc = 0.5; //contstant 0.4

    public final double kP_Vel = 0.06;
    public final double kI_Vel = 0.0008;
    public final double kD_Vel = 0.0000;
    public final double kIz_Vel = 4;
    public final double kFF_Vel = 0.0;


    

  

    public final double kMaxOutput_in = 0.4; //arm up
    public final double kMinOutput_in = 0.4;//arm down

    // public final double kMaxOutput_out = 0.4; //arm up
    // public final double kMinOutput_out = 0.3;//arm down

    // public final double kMaxOutput_in = 0.0; //arm up
    // public final double kMinOutput_in = 0.0;//arm down

    // public final double kMaxOutput_out = 0.0; //arm up
    // public final double kMinOutput_out = 0.0;//arm down
    

    public final double maxRPM = 5;
    public final double maxAccel = 1.0;
    public final double allowedErr = 0.5;
    public final double closedRR  = 0.5;

    public int kSlotIdx_Pos = 0;
    public int kSlotIdx_Vel = 1;

    public double maxVelocity= 400; 

    public double vSetPos = 0;


    public Raise() {
      m_raiseMotor = new CANSparkMax(21,MotorType.kBrushless);
      m_Encoder = m_raiseMotor.getEncoder();
      m_PID = m_raiseMotor.getPIDController();

      m_raiseMotor.setSoftLimit(SoftLimitDirection.kForward, (float) 0);
      m_raiseMotor.setSoftLimit(SoftLimitDirection.kReverse, (float)-Arm.k_rangeLeanglePos);

      m_raiseMotor.setIdleMode(IdleMode.kBrake);
      m_raiseMotor.setSmartCurrentLimit(14, 50, 0);
      m_raiseMotor.setClosedLoopRampRate(closedRR);


      initPID();

    }

    // public double setArbituaryFF(double kFF_arb){
      
    // }

    public double mapInput(double position){
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
      
        
        m_PID.setReference(position, CANSparkMax.ControlType.kPosition,  0);
        return position;
      }

  private double baseraiseRate= 11000;
  private SlewRateLimiter raiseSmother = new SlewRateLimiter(baseraiseRate,-baseraiseRate,0);
  public void updatesmoother(double position){
    
    double raiseRate = baseraiseRate*(1-(position/Arm.k_rangeExtentPos))+1500;
    raiseSmother =      new SlewRateLimiter(raiseRate,-raiseRate,0);
    raiseSmother.reset(vSetPos);
        
  }
      public double setPosition(double position, int slot, double feedforward){
        // System.out.println(feedforward);
        position = raiseSmother.calculate(position);
          vSetPos = position;
        m_PID.setReference(position, CANSparkMax.ControlType.kPosition,  0, feedforward);
        return position;
      }
      public double setPositionAuto(double position, int slot, double feedforward){
        // System.out.println(feedforward);
        // position = raiseSmother.calculate(position);
          vSetPos = position;
        m_PID.setReference(position, CANSparkMax.ControlType.kPosition,  0, feedforward);
        return position;
      }

      public double setVelocity(double velocityRatio,double arbFF){
        double velocity = velocityRatio * maxVelocity;
        m_PID.setReference(velocity, CANSparkMax.ControlType.kVelocity, 1,arbFF);
        vSetPos = velocity;
        return velocity;
      }

      public double setMotion(double position , int slot){
        
        m_PID.setReference(position, CANSparkMax.ControlType.kSmartMotion, 2);
        return position;
      }
      
      public double setMotionOB(double position , int slot){
        
        m_PID.setReference(position, CANSparkMax.ControlType.kSmartMotion, 2);
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
       @Log(name = "EMtr Temp" ,tabName = "TargetSelector", rowIndex = 4 , columnIndex = 0)
       @Log(name = "Mtr Temp" ,tabName = "Raise", rowIndex = 3 , columnIndex = 6)
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
        //  public double getPercentError(){
        //       return (m_Encoder.getPosition() - vSetPos)/Arm.k_rangeExtentPos*100;
        //  }
        @Log.Graph (name = "Velocity", tabName = "Raise", rowIndex = 3, columnIndex = 6)
        public double getVelocity(){
          return m_Encoder.getVelocity();
        }
        @Log.Graph (name = "Velocity Error", tabName = "Raise", rowIndex = 3, columnIndex = 9)
        public double getVelocityError(){
          return (m_Encoder.getVelocity() - vSetPos)/maxVelocity*100;
        }

        //  @Config (name = "P_up_in", tabName = "Raise", defaultValueNumeric = kP_Pos, rowIndex = 0, columnIndex = 0)
        void setP_up_in(double p){
          m_PID.setP(p , kSlotIdx_Pos);
        }
        // @Config (name = "I_up_in", tabName = "Raise", defaultValueNumeric = kI_Pos , rowIndex = 0, columnIndex = 1)
        void setI_up_in(double i){
          m_PID.setI(i, kSlotIdx_Pos);
        }
        // @Config (name = "D_up_in", tabName = "Raise", defaultValueNumeric = kD_Pos, rowIndex = 0, columnIndex = 2)
        void setD_up_in(double d){
          m_PID.setD(d, kSlotIdx_Pos);
        }
        // @Config (name = "Iz_up_in", tabName = "Raise", defaultValueNumeric = kIz_Pos, rowIndex = 0, columnIndex = 3)
        void setIz_up_in(double iz){
          m_PID.setIZone(iz, kSlotIdx_Pos);
        }
        // @Config (name = "F_up_in", tabName = "Raise", defaultValueNumeric = kFF_Pos, rowIndex = 0, columnIndex = 4)
        void setF_up_in(double f){
          m_PID.setFF(f, kSlotIdx_Pos);
        }
        @Config (name = "P_vel", tabName = "Raise", defaultValueNumeric = kP_Vel, rowIndex = 1, columnIndex = 0)  
        void setP_down_in(double p){
          m_PID.setP(p , kSlotIdx_Vel);
        }
        @Config (name = "I_vel", tabName = "Raise", defaultValueNumeric = kI_Vel, rowIndex = 1, columnIndex = 1)   

        void setI_down_in(double i){
          m_PID.setI(i, kSlotIdx_Vel);
        }
        @Config (name = "D_vel", tabName = "Raise", defaultValueNumeric = kD_Vel, rowIndex = 1, columnIndex = 2)
        void setD_down_in(double d){
          m_PID.setD(d, kSlotIdx_Vel);
        }
        @Config (name = "Iz_vel", tabName = "Raise", defaultValueNumeric = kIz_Vel, rowIndex = 1, columnIndex = 3)
        void setIz_down_in(double iz){
          m_PID.setIZone(iz, kSlotIdx_Vel);
        }
        @Config (name = "F_vel", tabName = "Raise", defaultValueNumeric = kFF_Vel, rowIndex = 1, columnIndex = 4)
        void setF_down_in(double f){
          m_PID.setFF(f, kSlotIdx_Vel);
        }
        @Config (name = "Raise ff_arb", tabName = "Raise", defaultValueNumeric = kFF_arbeDefault, rowIndex = 1, columnIndex = 5)
void setArbFFe(double f){
  karb_ffe = f;
} 
@Config (name = "Raise ff_arbC", tabName = "Raise", defaultValueNumeric = kFF_arbCDefault, rowIndex = 1, columnIndex = 6)
void setArbFFC(double f){
  karb_ffc = f;
}
@Config (name = "Raise ff_arbA", tabName = "Raise", defaultValueNumeric = kFF_arbADefault, rowIndex = 1, columnIndex = 7)
void setArbFFa(double f){
  karb_ffa = f;
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
  



        // @Config (name = "MaxOutput_in", tabName = "Raise"  ,rowIndex = 2, columnIndex = 0, defaultValueNumeric = kMaxOutput_in)
        void setMaxOutput_in(@Config(defaultValueNumeric = kMaxOutput_in) double upPower ,@Config(defaultValueNumeric = kMinOutput_in)  double downPower){
          m_PID.setOutputRange(-downPower, upPower, kSlotIdx_Pos);
          m_PID.setOutputRange(-downPower, upPower, kSlotIdx_Vel);

          
        }
        

        // @Config (name = "ClosedRR", tabName = "Raise", defaultValueNumeric = closedRR , rowIndex = 1, columnIndex = 5)
        void setClosedRR(double rr){
          m_raiseMotor.setClosedLoopRampRate(rr);
        }
    
        // @Log (name = "Raise Voltage", tabName = "Raise FF" , rowIndex = 0, columnIndex = 0)
        public double getVoltage(){
          return m_raiseMotor.getBusVoltage()*m_raiseMotor.getAppliedOutput();
        
        }
        //No need to worry aout max inpt (motor will always be fighting gravity)



     private void initPID(){


      m_PID.setP(kP_Pos, kSlotIdx_Pos);
      m_PID.setI(kI_Pos, kSlotIdx_Pos);
      m_PID.setD(kD_Pos, kSlotIdx_Pos);
      m_PID.setIZone(kIz_Pos, kSlotIdx_Pos);
      m_PID.setFF(kFF_Pos, kSlotIdx_Pos);
      m_PID.setOutputRange(-kMinOutput_in, kMaxOutput_in, kSlotIdx_Pos);

      m_PID.setP(kP_Vel, kSlotIdx_Vel);
      m_PID.setI(kI_Vel, kSlotIdx_Vel);
      m_PID.setD(kD_Vel, kSlotIdx_Vel);
      m_PID.setIZone(kIz_Vel, kSlotIdx_Vel);
      m_PID.setFF(kFF_Vel, kSlotIdx_Vel);
      m_PID.setOutputRange(-kMinOutput_in, kMaxOutput_in, kSlotIdx_Vel);

     

      m_PID.setSmartMotionMaxVelocity(maxRPM, kSlotIdx_Pos);
      m_PID.setSmartMotionMaxVelocity(maxRPM, kSlotIdx_Vel);
  
      m_PID.setSmartMotionMaxAccel(maxAccel, kSlotIdx_Pos);
      m_PID.setSmartMotionMaxAccel(maxAccel, kSlotIdx_Vel);
    

      m_PID.setSmartMotionAllowedClosedLoopError(allowedErr, kSlotIdx_Pos);
      m_PID.setSmartMotionAllowedClosedLoopError(allowedErr, kSlotIdx_Vel);
   



    }
    

}
