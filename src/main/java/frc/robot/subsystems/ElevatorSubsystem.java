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
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.RobotState;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.DriveConstants;
import frc.robot.Constants.ElevatorConstants;
import frc.robot.Constants.NEOMotorsConstants;

public class ElevatorSubsystem extends SubsystemBase implements MovableSubsystem {
  private final SparkMax leftMotorLeader;
  private final SparkMax rightMotorFollow;
  // private final SparkClosedLoopController controller;
  private final RelativeEncoder elevatorEncoder;
  private final MunecaSubsystem munecaSubsystem; // Referencia a la muñeca
  private double targetPosition = 0;
  private PIDController pidFeedback = new PIDController(0.02, 0.0, 0.0);

  private static final double UPPER_LIMIT = 87.0;
  private static final double LOWER_LIMIT = 0.0;
  private static final double MUNECA_THRESHOLD = -18; // Límite de la muñeca
  private static final double RESTRICTED_LOWER_LIMIT = 30; // Restricción del elevador cuando la muñeca está baja
  private static final double MAX_OUTPUT = 0.3;

  public static final String DASH_POS = "Elevador/Posicion";
  public static final String DASH_TARGET = "Elevador/Target";
  public static final String DASH_RESET_ENCODER = "Elevador/Reset Encoder";
  public static final String DASH_ELEVATOR_WARNING = "Elevador Restricción";

  public ElevatorSubsystem(MunecaSubsystem munecaSubsystem) {
    this.munecaSubsystem = munecaSubsystem;

    leftMotorLeader = new SparkMax(ElevatorConstants.LEFT_LEADER_CAN_ID, MotorType.kBrushless);
    rightMotorFollow = new SparkMax(ElevatorConstants.RIGHT_FOLLOW_CAN_ID, MotorType.kBrushless);

    leftMotorLeader.setCANTimeout(DriveConstants.CAN_TIMEOUT);
    rightMotorFollow.setCANTimeout(DriveConstants.CAN_TIMEOUT);
    // controller = leftMotorLeader.getClosedLoopController();
    leftMotorLeader.configure(getLeaderConfig(), ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
    rightMotorFollow.configure(getFollowConfig(), ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);


    configPIDController();

    elevatorEncoder = leftMotorLeader.getEncoder();
    reset();
    SmartDashboard.putData(DASH_RESET_ENCODER, new InstantCommand(this::reset));
  }
  
  public void configPIDController() {
    pidFeedback.setIZone(2.0);
    //pidFeedback.setIntegratorRange(0.05, 0.1);
    pidFeedback.setTolerance(1.0);
  }

  private SparkBaseConfig getFollowConfig() {
    return new SparkMaxConfig()
      .follow(leftMotorLeader, true)
      .idleMode(SparkBaseConfig.IdleMode.kBrake);
  }

  private static SparkMaxConfig getLeaderConfig() {
    SparkMaxConfig leaderConfig = new SparkMaxConfig();

    // leaderConfig.closedLoop
    //   .p(0.02)
    //   .i(0)
    //   .d(0)
    //   .feedbackSensor(ClosedLoopConfig.FeedbackSensor.kPrimaryEncoder)
    //   .maxMotion
    //   .maxVelocity(1)
    //   .maxAcceleration(0.05)
    //   .allowedClosedLoopError(0.5);

    leaderConfig
      .voltageCompensation(NEOMotorsConstants.VOLTAGE_COMPENSATION)
      .smartCurrentLimit(NEOMotorsConstants.CURRENT_LIMIT)
      .idleMode(SparkBaseConfig.IdleMode.kBrake)
      .inverted(false);

    return leaderConfig;
  }

  @Override
  public void periodic() {
    SmartDashboard.putNumber(DASH_POS, getActualPosition());
    SmartDashboard.putNumber(DASH_TARGET, targetPosition);

    if (isRestrictedArea() && RobotState.isTeleop() && RobotState.isEnabled()) {
      SmartDashboard.putString(DASH_ELEVATOR_WARNING, "⚠ Elevador restringido! Muñeca muy baja.");
    }

    double output = pidFeedback.calculate(this.getActualPosition(), this.getTargetPosition());
    output = Math.max(output, MAX_OUTPUT);
    leftMotorLeader.set(output);

    // controller.setReference(this.position, SparkBase.ControlType.kPosition);
  }

  @Override
  public double getActualPosition() {
    return elevatorEncoder.getPosition();
  }

  @Override
  public void move(double position) {
    double minLimit = isRestrictedArea() ? RESTRICTED_LOWER_LIMIT : LOWER_LIMIT;
    this.targetPosition = MathUtil.clamp(position, minLimit, UPPER_LIMIT);
  }

  @Override
  public double getTargetPosition() {
    return targetPosition;
  }

  @Override
  public void reset() {
    this.targetPosition = 0;
    elevatorEncoder.setPosition(0);
    // controller.setReference(this.position, SparkBase.ControlType.kPosition);
  }

  public boolean isRestrictedArea() {
    return munecaSubsystem.getActualPosition() < MUNECA_THRESHOLD;
  }
}