package frc.robot.subsystems;

import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.SparkBaseConfig;
import com.revrobotics.spark.config.SparkMaxConfig;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.Constants.DriveConstants;
import frc.robot.Constants.ElevadorConstants;
import frc.robot.Constants.NEOMotorsConstants;
import frc.robot.Constants.LimitesEncoders;

public class ElevatorSubsystem extends SubsystemBase {
  private final SparkMax leftMotorLeader;
  private final SparkMax rightMotorFollow;
  private final RelativeEncoder elevatorEncoder;

  private static final double SLOWDOWN_RANGE = 20.0;
  private static final double UPPER_LIMIT = 85.0;
  private static final double LOWER_LIMIT = 0.0;
  public static final String DASH_ELEVATOR_POS = "Elevador Posicion";
  public static final String DASH_RESET_ELEVATOR_ENCODER = "Reiniciar Encoder Elevador";

  public ElevatorSubsystem() {
    leftMotorLeader = new SparkMax(ElevadorConstants.LEFT_ELEVATOR_LEADER_MOTOR_ID, MotorType.kBrushless);
    rightMotorFollow = new SparkMax(ElevadorConstants.RIGHT_ELEVATOR_FOLLOW_MOTOR_ID, MotorType.kBrushless);

    leftMotorLeader.setCANTimeout(DriveConstants.CAN_TIMEOUT);
    rightMotorFollow.setCANTimeout(DriveConstants.CAN_TIMEOUT);

    leftMotorLeader.configure(getLeaderConfig(), ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
    rightMotorFollow.configure(getFollowConfig(), ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);

    elevatorEncoder = leftMotorLeader.getEncoder();
    elevatorEncoder.setPosition(0);

    SmartDashboard.putData(DASH_RESET_ELEVATOR_ENCODER, new InstantCommand(() -> elevatorEncoder.setPosition(0)));
  }

  private SparkBaseConfig getFollowConfig() {
    return new SparkMaxConfig().follow(leftMotorLeader, true)
    .idleMode(SparkBaseConfig.IdleMode.kBrake); // Modo Brake para evitar caída
}


  private static SparkMaxConfig getLeaderConfig() {
    SparkMaxConfig leaderConfig = new SparkMaxConfig();

    leaderConfig.softLimit
      .forwardSoftLimitEnabled(true)
      .forwardSoftLimit(85)
      .reverseSoftLimitEnabled(true)
      .reverseSoftLimit(0);

    leaderConfig
      .voltageCompensation(NEOMotorsConstants.VOLTAGE_COMPENSATION_NEO)
      .smartCurrentLimit(NEOMotorsConstants.CURRENT_LIMIT_NEO)

      .idleMode(SparkBaseConfig.IdleMode.kBrake) //  Modo Brake para evitar caída
      .inverted(false);

    return leaderConfig;
  }

  @Override
  public void periodic() {
    SmartDashboard.putNumber(DASH_ELEVATOR_POS, getElevatorPosition());
  }

  public double getElevatorPosition() {
    return elevatorEncoder.getPosition();
  }

  public void moveElevator(double speed) {
    double currentPosition = getElevatorPosition();

    boolean inLowerSlowdownZone = currentPosition <= LOWER_LIMIT + SLOWDOWN_RANGE;
    boolean inUpperSlowdownZone = currentPosition >= UPPER_LIMIT - SLOWDOWN_RANGE;
    if (speed < 0 && inLowerSlowdownZone) speed = speed * getLowerSlowdownFactor(currentPosition);
    if (speed > 0 && inUpperSlowdownZone) speed = speed * getUpperSlowdownFactor(currentPosition);
    leftMotorLeader.set(speed);
  }

  public double getUpperSlowdownFactor(double currentPosition) {
    double resultado = Math.max((UPPER_LIMIT - currentPosition) / SLOWDOWN_RANGE, 0);
    if (resultado == 0 ) return 0;
    if (resultado <= Constants.LimitesEncoders.LimiteFuerzaAceleracion) return Constants.LimitesEncoders.LimiteFuerzaAceleracion;
    return resultado;
  }

  public double getLowerSlowdownFactor(double currentPosition) {
    double resultado = Math.max((currentPosition - LOWER_LIMIT) / SLOWDOWN_RANGE, 0);
    if (resultado == 0 ) return 0;
    if (resultado <= LimitesEncoders.LimiteFuerzaAceleracion) return LimitesEncoders.LimiteFuerzaAceleracion;
    return resultado;
  }

  public void ResetElevador() {
    if (getElevatorPosition() > 0) {
        moveElevator(-0.2); // Baja el elevador lentamente
    }
}

}