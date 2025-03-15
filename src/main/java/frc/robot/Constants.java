package frc.robot;

public final class Constants {
  public static final class DriveConstants {
    public static final int LEFT_LEADER_CAN_ID = 1;
    public static final int LEFT_FOLLOW_CAN_ID = 2;
    public static final int RIGHT_LEADER_CAN_ID = 3;
    public static final int RIGHT_FOLLOW_CAN_ID = 4;

    public static final int DRIVE_MOTOR_CURRENT_LIMIT = 60;
    public static final double AXIS_VELOCIDAD_LIMIT = 0.5;
    public static final double AXIS_GIRO_LIMIT = 0.5;
    public static final double BUMPER_ACEL_LIMIT = 0.5;
    public static final double ACEL_AUMENTO = 0.15;
    public static final double DEAD_POINT = 0.0;

    public static final int CAN_TIMEOUT = 250;
  }

  public static final class OperatorConstants {
    public static final int DRIVER = 0;
    public static final int OPERATOR = 1;
  }

  public static final class EngancheConstants {
    public static final int CAN_ID = 11;
  }

  public static final class ElevatorConstants {
    public static final int LEFT_LEADER_CAN_ID = 5;
    public static final int RIGHT_FOLLOW_CAN_ID = 6;
    public static final int L0 = 0;
    public static final double L1 = 16.3;
    public static final double L2 = 41.1;
    public static final int L3 = 85;
  }

  public static final class NEOMotorsConstants {
    public static final int CURRENT_LIMIT = 40;
    public static final double VOLTAGE_COMPENSATION = 12.0;
  }

  public static final class DeadZone {
    public static final double ELEVATOR = 0.05;
    public static final double MOVEMENT = 0.05;
  }

  public static final class MunecaConstants {
    public static final int CAN_ID = 9;
  }

  public static final class HandConstants {
    public static final int INTERIOR_CAN_ID = 10;
    public static final int SIDE_CAN_ID = 8;
  }
}