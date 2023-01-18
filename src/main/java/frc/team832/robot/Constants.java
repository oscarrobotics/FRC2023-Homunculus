package frc.team832.robot;

import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.util.Units;

/**
 * The Constants class provides a convenient place for teams to hold robot-wide numerical or boolean
 * constants. This class should not be used for any other purpose. All constants should be declared
 * globally (i.e. public static). Do not put anything functional in this class.
 *
 * <p>
 * It is advised to statically import this class (or one of its inner classes) wherever the
 * constants are needed, to reduce verbosity.
 */
public final class Constants {

    private Constants() {}

    public static final double LB_IN2_TO_KG_M2 = 0.000292639653;

    public static final int RPD_CAN_ID = 0;
    public static final int RPH_CAN_ID = 0;

    public static final class RobotConstants {
        public static final double RobotBumperedSizeMeters = Units.inchesToMeters(32.25);
        public static final Translation2d RobotBumperEdgeFromCenter =
                new Translation2d(RobotBumperedSizeMeters / 2, RobotBumperedSizeMeters / 2);
        public static final double RobotHalfBumperedWidth = RobotBumperedSizeMeters / 2;
    }

    public static final class DriveTrainConstants {

        /** CAN IDs **/
        public static final int LEFT_MASTER_TALON_ID = 1;
        public static final int LEFT_SLAVE_TALON_ID = 2;
        public static final int RIGHT_MASTER_TALON_ID = 3;
        public static final int RIGHT_SLAVE_TALON_ID = 4;
        public static final int PIGEON_ID = 0;
        /** Power **/
        public static final int CURRENT_LIMIT = 55;

        /** Mechanical Characteristics **/
        public static final Motor MOTOR = Motor.kFalcon500;
        public static final double WHEEL_DIAMETER_INCHES = 0.0;
        public static final double WHEELBASE_INCHES = 0.0;
        public static final double WHEELBASE_METERS = Units.inchesToMeters(WHEELBASE_INCHES);
        public static final double TRACKWIDTH_METERS = 0.0;
        public static final double MASS_KG = Units.lbsToKilograms(0.0);
        public static final double MOI_KGM2 = 0.0;
        // TODO: set up powertrain + gearbox reduction

        /** System Control Values **/
        public static final double RIGHT_KS = 0.0;
        public static final double RIGHT_KV = 0.0;
        public static final double RIGHT_KA = 0.0;
        public static final SimpleMotorFeedforward RIGHT_FEEDFORWARD =
                new SimpleMotorFeedforward(RIGHT_KS, RIGHT_KV, RIGHT_KA);
        public static final double RIGHT_KP = 0.0;

        public static final double LEFT_KS = 0.0;
        public static final double LEFT_KV = 0.0;
        public static final double LEFT_KA = 0.0;
        public static final SimpleMotorFeedforward LEFT_FEEDFORWARD =
                new SimpleMotorFeedforward(LEFT_KS, LEFT_KV, LEFT_KA);
        public static final double LEFT_KP = 0.0;

        public static final double ANGULAR_KS = 0.0;
        public static final double ANGULAR_KV = 0.0;
        public static final double ANGULAR_KA = 0.0;
        public static final double ANGULAR_KP = 0.0;

        // TODO: Set up pathweaver constants

    }

    public static final class IntakeConstants {
        /** CAN IDs **/
        public static final int INTAKE_MOTOR_TALON_ID = 0;

        /** Power **/
        public static final int CURRENT_LIMIT = 25;

        /** Mechanical Characteristics */
        public static final double INTAKE_POWER = 0.0;
        public static final double OUTTAKE_POWER = -0.0;
        public static final double INTAKE_REDUCTION = 0.0 / 0.0;

        /** System Control Values */
        public static final double KP = 0.0;
        public static final double KS = 0.0;
        public static final double KV = Motor.kFalcon500.KvRPMPerVolt;
        public static final SimpleMotorFeedforward FEEDFORWARD = new SimpleMotorFeedforward(KS, KV);
    }

    public static final class ArmConstants { // Constants are very much subject to change. Design
                                             // idea remains rudimentary

        /** CAN IDs */
        public static final int SHOULDER_MOTOR_TALON_ID = 0;
        public static final int ARM_MOTOR_TALON_ID = 0;
        public static final int CLAW_MOTOR_TALON_ID = 0;

        /** System Control Values */
        public static final double SHOULDER_KP = 0.0;
        public static final double SHOULDER_KS = 0.0;
        public static final double SHOULDER_KV = 0.0;
        public static final double SHOULDER_KA = 0.0;
        public static final double ARM_KP = 0.0;
        public static final double ARM_KS = 0.0;
        public static final double ARM_KV = 0.0;
        public static final double ARM_KA = 0.0;
        public static final double CLAW_KP = 0.0;
        public static final double CLAW_KS = 0.0;
        public static final double CLAW_KV = 0.0;
        public static final double CLAW_KA = 0.0;
        public static final double SHOULDER_MOTOR_OFFSET = 0.0;
        public static final double ARM_MOTOR_OFFSET = 0.0;
        public static final double CLAW_MOTOR_OFFSET = 0.0;

        /** Mechanical Characteristics **/
        public static final double ARM_LENGTH_INCHES = 0.0;
        public static final double ARM_LENGTH_METERS = Units.inchesToMeters(ARM_LENGTH_INCHES);
        public static final double ARM_MAX_EXTEND_POS = 0.0; // subject to change based on design
    }

    public static class VisionConstants {
        public static final double CAMERA_HEIGHT_METERS = 0.0;
        public static final double CAMERA_PITCH_RADIANS = 0.0;
        public static final double TARGET_HEIGHT_METERS = 0.0;
    }

    /** Nested motor class to maintain PID constants and RPM */
    public static class Motor extends DCMotor {

        public final double KvRPMPerVolt;
        public final double freeSpeedRPM;

        public static Motor fromDCMotor(DCMotor wpilibMotor) {
            return new Motor(wpilibMotor.nominalVoltageVolts, wpilibMotor.stallTorqueNewtonMeters,
                    wpilibMotor.stallCurrentAmps, wpilibMotor.freeCurrentAmps,
                    Units.radiansPerSecondToRotationsPerMinute(wpilibMotor.freeSpeedRadPerSec));
        }

        public Motor(double nominalVoltageVolts, double stallTorqueNewtonMeters,
                double stallCurrentAmps, double freeCurrentAmps, double freeSpeedRPM) {
            super(nominalVoltageVolts, stallTorqueNewtonMeters, stallCurrentAmps, freeCurrentAmps,
                    Units.rotationsPerMinuteToRadiansPerSecond(freeSpeedRPM), 1);

            this.freeSpeedRPM = Units.radiansPerSecondToRotationsPerMinute(freeSpeedRadPerSec);
            this.KvRPMPerVolt = Units.radiansPerSecondToRotationsPerMinute(KvRadPerSecPerVolt);
        }

        public static final Motor kFalcon500 = fromDCMotor(DCMotor.getFalcon500(1));
        public static final Motor kNEO = fromDCMotor(DCMotor.getNEO(1));
        public static final Motor kNEO550 = fromDCMotor(DCMotor.getNeo550(1));
    }



}


