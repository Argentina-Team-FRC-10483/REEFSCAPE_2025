package frc.robot;

public final class Constants {
  public static final class DriveConstants {
    public static final int LEFT_MOVEMENT_LEADER_MOTOR_ID = 1;
    public static final int LEFT_MOVEMENT_FOLLOW_MOTOR_ID = 2;
    public static final int RIGHT_MOVEMENT_LEADER_MOTOR_ID = 3;
    public static final int RIGHT_MOVEMENT_FOLLOW_MOTOR_ID = 4;

    public static final int DRIVE_MOTOR_CURRENT_LIMIT = 60;
    public static final double AXIS_VELOCIDAD_LIMIT = 0.5;
    public static final double AXIS_GIRO_LIMIT = 0.5;
    public static final double BUMPER_ACEL_LIMIT = 0.5;
    public static final double ACEL_AUMENTO = 0.15;
    public static final double DEAD_POINT = 0.0;
    public static final double DEAD_ZONE = 0.12;

    public static final int CAN_TIMEOUT = 250;
  }

  public static final class OperatorConstants {
    public static final int DRIVER_CONTROLLER_PORT = 0;
    public static final int OPERADOR_CONTROLLER_PORT = 1;
  }

  public static final class AlgaeIntakeConstants {
    public static final int RodilloMotor_ID = 7;
    public static final int RodilloMotor_LIMITE = 60;
    public static final double RodilloMotor_CompVolt = 10;
    public static final double RodilloMotor_ValorExp = 0.44;
  }

  public static final class EngancheContants {
    public static final int MotorEnganche_ID = 8;
    public static final int MotorEnganche_LIMITE = 40;
    public static final double MotorEnganche_CompVolt = 10;
    public static final double MotorEnganche_ValorExp = 0.44;
  }

  public static final class ElevadorConstants {
    public static final int LEFT_ELEVATOR_LEADER_MOTOR_ID = 5;
    public static final int RIGHT_ELEVATOR_FOLLOW_MOTOR_ID = 6;
    public static final int CURRENT_LIMIT = 40;
    public static final double VOLTAGE_COMPENSATION = 12.0;
  }

  public static final class DeadZone {
    public static final double ElevadorDeadZone = 0.15;
    public static final double MovimientoDeadZone = 0.05;
  }
}
