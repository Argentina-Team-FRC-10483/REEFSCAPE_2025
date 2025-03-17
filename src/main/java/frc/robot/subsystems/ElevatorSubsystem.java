package frc.robot.subsystems;

import com.revrobotics.spark.SparkBase;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.ClosedLoopConfig;
import com.revrobotics.spark.config.SparkBaseConfig;
import com.revrobotics.spark.config.SparkMaxConfig;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.DriveConstants;
import frc.robot.Constants.ElevatorConstants;
import frc.robot.Constants.NEOMotorsConstants;

public class ElevatorSubsystem extends SubsystemBase implements MovableSubsystem {
  private final SparkMax leftMotorLeader;
  private final SparkMax rightMotorFollow;
  private final SparkClosedLoopController controller;
  private final RelativeEncoder elevatorEncoder;
  private double position = 0;

  private static final double UPPER_LIMIT = 89.0;
  private static final double LOWER_LIMIT = 0.0;
  public static final String DASH_POS = "Elevador/Posicion";
  public static final String DASH_TARGET = "Elevador/Target";
  public static final String DASH_RESET_ENCODER = "Elevador/Reset Encoder";

  public ElevatorSubsystem() {
    leftMotorLeader = new SparkMax(ElevatorConstants.LEFT_LEADER_CAN_ID, MotorType.kBrushless);
    rightMotorFollow = new SparkMax(ElevatorConstants.RIGHT_FOLLOW_CAN_ID, MotorType.kBrushless);

    leftMotorLeader.setCANTimeout(DriveConstants.CAN_TIMEOUT);
    rightMotorFollow.setCANTimeout(DriveConstants.CAN_TIMEOUT);
    controller = leftMotorLeader.getClosedLoopController();
    leftMotorLeader.configure(getLeaderConfig(), ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
    rightMotorFollow.configure(getFollowConfig(), ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);

    elevatorEncoder = leftMotorLeader.getEncoder();
    reset();
    SmartDashboard.putData(DASH_RESET_ENCODER, new InstantCommand(this::reset));
  }

  private SparkBaseConfig getFollowConfig() {
    return new SparkMaxConfig()
      .follow(leftMotorLeader, true)
      .idleMode(SparkBaseConfig.IdleMode.kBrake); // Modo Brake para evitar caída
  }

  private static SparkMaxConfig getLeaderConfig() {
    SparkMaxConfig leaderConfig = new SparkMaxConfig();

    leaderConfig.closedLoop
      .p(0.02)
      .i(0)
      .d(0)
      .feedbackSensor(ClosedLoopConfig.FeedbackSensor.kPrimaryEncoder)
      .maxMotion
      .maxVelocity(0.4)
      .maxAcceleration(0.001)
      .allowedClosedLoopError(0.5);

    leaderConfig
      .voltageCompensation(NEOMotorsConstants.VOLTAGE_COMPENSATION)
      .smartCurrentLimit(NEOMotorsConstants.CURRENT_LIMIT)
      .idleMode(SparkBaseConfig.IdleMode.kBrake) //  Modo Brake para evitar caída
      .inverted(false);

    return leaderConfig;
  }

  @Override
  public void periodic() {
    SmartDashboard.putNumber(DASH_POS, getActualPosition());
    SmartDashboard.putNumber(DASH_TARGET, position);
    controller.setReference(this.position, SparkBase.ControlType.kPosition);
  }

  @Override
  public double getActualPosition() {
    return elevatorEncoder.getPosition();
  }

  @Override
  public void move(double position) {
    this.position = MathUtil.clamp(position, LOWER_LIMIT, UPPER_LIMIT);
  }

  @Override
  public double getTargetPosition() {
    return position;
  }

  @Override
  public void reset() {
    this.position = 0;
    elevatorEncoder.setPosition(0);
    controller.setReference(this.position, SparkBase.ControlType.kPosition);
  }
}