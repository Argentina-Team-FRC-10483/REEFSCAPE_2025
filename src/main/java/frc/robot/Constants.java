package frc.robot;

public final class Constants {
  public static final class DriveConstants {
    public static final int LEFT_LEADER_CAN_ID = 1;
    public static final int LEFT_FOLLOW_CAN_ID = 2;
    public static final int RIGHT_LEADER_CAN_ID = 3;
    public static final int RIGHT_FOLLOW_CAN_ID = 4;

    public static final int DRIVE_MOTOR_CURRENT_LIMIT = 60;
    public static final double AXIS_SPEED_LIMIT = 0.5;
    public static final double AXIS_TURN_LIMIT = 0.5;
    public static final double BUMPER_ACCEL_LIMIT = 0.5;
    public static final double ACCEL_INCREASE = 0.45;
    public static final double DEAD_POINT = 0.0;

    public static final int CAN_TIMEOUT = 250;
  }

  public static final class OperatorConstants {
    public static final int DRIVER_CONTROLLER_PORT = 0;
    public static final int OPERATOR_CONTROLLER_PORT = 1;
  }

  public static final class HangingConstants {
    public static final int CAN_ID = 7;
  }

  public static final class ElevatorConstants {
    public static final int LEFT_LEADER_CAN_ID = 5;
    public static final int RIGHT_FOLLOW_CAN_ID = 6;
    public static final int L0 = 0;
    public static final double L1 = 16.3;
    public static final double L2 = 41.1;
    public static final int L3 = 86;
    public static final boolean ENABLE_ELEVATOR_SETPOINTS = false;
  }

  public static final class NEOMotorsConstants {
    public static final int CURRENT_LIMIT = 40;
    public static final double VOLTAGE_COMPENSATION = 12.0;
  }

  public static final class DeadZone {
    public static final double ELEVATOR = 0.1;
    public static final double MOVEMENT = 0.1;
  }

  public static final class ArmConstants {
    public static final int CAN_ID = 9;
  }

  public static final class HandConstants {
    public static final int SIDE_CAN_ID = 11;
  }
}