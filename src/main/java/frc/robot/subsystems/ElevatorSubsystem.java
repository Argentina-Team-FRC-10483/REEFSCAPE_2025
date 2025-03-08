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
import frc.robot.Constants.DriveConstants;
import frc.robot.Constants.ElevadorConstants;
import frc.robot.Constants.NEOMotorsConstants;
import frc.robot.utils.MotorSlowdownLimits;

public class ElevatorSubsystem extends SubsystemBase {
  private final SparkMax leftMotorLeader;
  private final SparkMax rightMotorFollow;
  private final RelativeEncoder elevatorEncoder;
  private final MotorSlowdownLimits slowdownLimits;

  public static final double SLOWDOWN_RANGE = 20.0;
  private static final double UPPER_LIMIT = 86.0;
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
    slowdownLimits = new MotorSlowdownLimits(
      LOWER_LIMIT,
      UPPER_LIMIT,
      SLOWDOWN_RANGE,
      this::getElevatorPosition,
      leftMotorLeader::set
    );
  }

  private SparkBaseConfig getFollowConfig() {
    return new SparkMaxConfig().follow(leftMotorLeader, true)
      .idleMode(SparkBaseConfig.IdleMode.kBrake); // Modo Brake para evitar caída
  }

  private static SparkMaxConfig getLeaderConfig() {
    SparkMaxConfig leaderConfig = new SparkMaxConfig();

    leaderConfig.softLimit
      .forwardSoftLimitEnabled(true)
      .forwardSoftLimit(86)
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
    slowdownLimits.move(speed);
  }
}
