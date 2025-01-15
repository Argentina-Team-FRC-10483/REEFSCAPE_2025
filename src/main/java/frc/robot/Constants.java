
package frc.robot;

public final class Constants {
  public static final class JoystickMappings{

    public static final int PinzaButtonApertura_ID = 6;
    public static final int PinzaAxisSuccion_ID = 2;
    public static final int PinzaAxisExpulsar_ID = 3;
  }
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
  }
  public static final class PinzaConstants {
  
    public static final int PinzaMotorArticulacionCentral_ID = 10;
    public static final int PinzaMotorRodilloIzquierdo_ID = 9;
    public static final int PinzaMotorRodilloDerecho_ID = 8;

  }
}
