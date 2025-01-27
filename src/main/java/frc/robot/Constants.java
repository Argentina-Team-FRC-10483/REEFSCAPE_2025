package frc.robot;

public final class Constants {
  public static final class DriveConstants {
    public static final int MotorMovimientoIzquierdoLider_ID = 1;
    public static final int MotorMovimientoIzquierdoSeguidor_ID = 2;
    public static final int MotorMovimientoDerechoLider_ID = 3;
    public static final int MotorMovimientoDerechoSeguidor_ID = 4;

    public static final int DRIVE_MOTOR_CURRENT_LIMIT = 60;

    public static final int CAN_TIMEOUT = 250;
  }
  public static final class OperatorConstants {
    public static final int DRIVER_CONTROLLER_PORT = 0;
    public static final int OPERADOR_CONTROLLER_PORT = 1;
  }

  public static final class AlgaeIntakeConstants {
    public static final int RodilloMotor_ID = 5;
    public static final int RodilloMotor_LIMITE = 60;
    public static final double RodilloMotor_CompVolt = 10;
    public static final double RodilloMotor_ValorExp = 0.44;
  }
  public static final class ElevadorConstants {
    public static final int MotorElevadorIzquierdoLider_ID = 5;
    public static final int MotorElevadorDerechoSeguidor_ID = 6;

    public static final int CURRENT_LIMIT = 40;
    public static final double VOLTAGE_COMPENSATION = 12.0;
  }
}
