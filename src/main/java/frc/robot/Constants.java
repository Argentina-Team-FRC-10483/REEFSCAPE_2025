package frc.robot;

public final class Constants {
  public static final class DriveConstants {
    public static final int LEFT_MOVEMENT_LEADER_MOTOR_ID = 1;
    public static final int LEFT_MOVEMENT_FOLLOW_MOTOR_ID = 2;
    public static final int RIGHT_MOVEMENT_LEADER_MOTOR_ID = 4;
    public static final int RIGHT_MOVEMENT_FOLLOW_MOTOR_ID = 3;

    public static final int DRIVE_MOTOR_CURRENT_LIMIT = 60;
    public static final double AXIS_VELOCIDAD_LIMIT = 0.5;
    public static final double AXIS_GIRO_LIMIT = 0.5;
    public static final double BUMPER_ACEL_LIMIT = 0.5;
    public static final double ACEL_AUMENTO = 0.3;
    public static final double DEAD_POINT = 0.0;
    public static final double DEAD_ZONE = 0.12;

    public static final int CAN_TIMEOUT = 250;
  }

  public static final class OperatorConstants {
    public static final int DRIVER_CONTROLLER_PORT = 0;
    public static final int OPERADOR_CONTROLLER_PORT = 1;
  }

  public static final class ArmConstants {
    public static final int ArmMotorShoulder_ID = 10;
  }

  public static final class PinzaConstants {
    //Motores
    public static final int PinzaMotorRodilloCentral_ID = 9;
    public static final int PinzaMotorRodilloDerecho_ID = 8;
  }

  public static final class AlgaeIntakeConstants {
    public static final int RodilloMotor_ID = 7;
    public static final double RodilloMotor_ValorExp = 0.44;
  }

  public static final class EngancheConstants {
    public static final int MOTOR_ENGANCHE_ID = 8;
  }

  public static final class ElevadorConstants {
    public static final int LEFT_ELEVATOR_LEADER_MOTOR_ID = 5;
    public static final int RIGHT_ELEVATOR_FOLLOW_MOTOR_ID = 6;
  }

  public static final class NEOMotorsConstants {
    public static final int CURRENT_LIMIT_NEO = 40;
    public static final double VOLTAGE_COMPENSATION_NEO = 12.0;
  }

  public static final class DeadZone {
    public static final double ElevadorDeadZone = 0.05;
    public static final double MovimientoDeadZone = 0.05;
  }

  public static final class LimitesEncoders {
    /**
     * This is used to add a minimum velocity for the motors when they are getting to a target.
     * That way if it is very close they don't do near 0 force which could damage them.
     */
    public static final double LimiteFuerzaAceleracion = 0.166666;
  }

  public static final class MunecaConstants {
    public static final int MUNECA_MOTOR_ID = 9;
  }

  public static final class HandConstants {
    public static final int ROD_INTERIOR_ID = 10;
    public static final int ROD_LATERALES_ID = 11;
  }
}