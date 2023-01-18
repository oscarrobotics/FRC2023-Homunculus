package frc.team832.robot.util;

import com.ctre.phoenix.motorcontrol.StatorCurrentLimitConfiguration;
import com.ctre.phoenix.motorcontrol.SupplyCurrentLimitConfiguration;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;


/* Unused WIP as of now */
public class CANTalonFX extends WPI_TalonFX {

  public static final int ENCODER_CPR = 2048;

  private final WPI_TalonFX _talon;
  private final int _canId;

  private SupplyCurrentLimitConfiguration inputCurrentConfig =
      new SupplyCurrentLimitConfiguration(true, 40, 0, 0);
  private StatorCurrentLimitConfiguration outputCurrentConfig =
      new StatorCurrentLimitConfiguration(true, 40, 0, 0);

  public CANTalonFX(int canId) {
    super(canId);
    _canId = canId;
    _talon = new WPI_TalonFX(canId);
  }
  // TODO: implement some interface that takes in electrical information

}
