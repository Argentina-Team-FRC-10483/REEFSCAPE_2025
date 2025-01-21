package frc.robot;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.signals.GravityTypeValue;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.units.Units;

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
  public static final class ArmConstants {
    public static final int ArmMotorShoulder_ID = 7;
  }
  public static final class PinzaConstants {
  
    public static final int PinzaMotorArticulacionCentral_ID = 10;
    public static final int PinzaMotorRodilloIzquierdo_ID = 9;
  }

  public static final class AlgaeIntakeConstants {
    public static final int RodilloMotor_ID = 5;
    public static final int RodilloMotor_LIMITE = 60;
    public static final double RodilloMotor_CompVolt = 10;
    public static final double RodilloMotor_ValorExp = 0.44;
  }
  public static class constElevator {
    public static TalonFXConfiguration ELEVATOR_CONFIG = new TalonFXConfiguration();
    static {
      ELEVATOR_CONFIG.MotorOutput.NeutralMode = NeutralModeValue.Brake;
      ELEVATOR_CONFIG.MotorOutput.Inverted = InvertedValue.Clockwise_Positive;

      ELEVATOR_CONFIG.SoftwareLimitSwitch.ForwardSoftLimitEnable = true;
      ELEVATOR_CONFIG.SoftwareLimitSwitch.ForwardSoftLimitThreshold = Units.Inches.of(66).in(Units.Inches);
      ELEVATOR_CONFIG.SoftwareLimitSwitch.ReverseSoftLimitEnable = true;
      ELEVATOR_CONFIG.SoftwareLimitSwitch.ReverseSoftLimitThreshold = Units.Inches.of(0)
          .in(Units.Inches);

      ELEVATOR_CONFIG.Slot0.GravityType = GravityTypeValue.Elevator_Static;
      // Elevator motors will provide feedback in INCHES the carriage has moved
      ELEVATOR_CONFIG.Feedback.SensorToMechanismRatio = 0.4545;
      ELEVATOR_CONFIG.Slot0.kG = 0.3;
      ELEVATOR_CONFIG.Slot0.kS = 0.4;
      // ELEVATOR_CONFIG.Slot0.kP = 1;
      ELEVATOR_CONFIG.Slot0.kP = 0.3;
    }

    public static final Distance CORAL_L1_HEIGHT = Units.Inches.of(9.039062);
    public static final Distance CORAL_L2_HEIGHT = Units.Inches.of(17.946289);
    public static final Distance CORAL_L3_HEIGHT = Units.Inches.of(33.742188);
    public static final Distance CORAL_L4_HEIGHT = Units.Inches.of(58.888916);
  }
  public static class mapElevator {
    public static final int MOTOR_ELEVADOR_IZQUIERDO_SEGUIDOR_CAN = 50;
    public static final int MOTOR_ELEVADOR_DERECHO_LIDER_CAN = 51;
  }
}
