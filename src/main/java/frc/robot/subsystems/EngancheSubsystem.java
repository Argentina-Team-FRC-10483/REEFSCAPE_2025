package frc.robot.subsystems;

import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.SparkBase;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.ClosedLoopConfig;
import com.revrobotics.spark.config.SparkBaseConfig;
import com.revrobotics.spark.config.SparkMaxConfig;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.DriveConstants;
import frc.robot.Constants.EngancheConstants;
import frc.robot.Constants.NEOMotorsConstants;

public class EngancheSubsystem extends SubsystemBase implements MovableSubsystem {
  private final SparkMax motor = new SparkMax(EngancheConstants.CAN_ID, MotorType.kBrushless);
  private final SparkClosedLoopController controller = motor.getClosedLoopController();
  private final RelativeEncoder engancheEncoder;
  private static final double UPPER_LIMIT = 10000;
  private static final double LOWER_LIMIT = -10000;
  public static final String DASH_MUNECA_POS = "Enganche/Pos";
  public static final String DASH_MUNECA_TARGET = "Enganche/Target";
  public static final String DASH_RESET_MUNECA_ENCODER = "Enganche/Reset Encoder";
  private double position;

  public EngancheSubsystem() {
    motor.setCANTimeout(DriveConstants.CAN_TIMEOUT);
    motor.configure(getLeaderConfig(), ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
    engancheEncoder = motor.getEncoder();
    reset();
    SmartDashboard.putData(DASH_RESET_MUNECA_ENCODER, new InstantCommand(this::reset));
  }

  private static SparkMaxConfig getLeaderConfig() {
    SparkMaxConfig leaderConfig = new SparkMaxConfig();

    leaderConfig.closedLoop
      .p(0.02)
      .i(0.00004)
      .iZone(1.7)
      .d(0)
      .feedbackSensor(ClosedLoopConfig.FeedbackSensor.kPrimaryEncoder)
      .maxMotion
      .maxVelocity(2000)
      .allowedClosedLoopError(0.25)
      .maxAcceleration(2000);

    leaderConfig
      .voltageCompensation(NEOMotorsConstants.VOLTAGE_COMPENSATION)
      .smartCurrentLimit(NEOMotorsConstants.CURRENT_LIMIT)
      .idleMode(SparkBaseConfig.IdleMode.kBrake);

    return leaderConfig;
  }

  @Override
  public void reset(){
    engancheEncoder.setPosition(0);
    position = 0;
    controller.setReference(this.position, SparkBase.ControlType.kMAXMotionPositionControl);
  }

  @Override
  public void periodic() {
    SmartDashboard.putNumber(DASH_MUNECA_POS, getActual());
    SmartDashboard.putNumber(DASH_MUNECA_TARGET, position);
    controller.setReference(this.position, SparkBase.ControlType.kMAXMotionPositionControl);
  }

  @Override
  public double getActual() {
    return engancheEncoder.getPosition();
  }

  @Override
  public double getTarget() {
    return position;
  }

  @Override
  public void move(double position) {
    this.position = MathUtil.clamp(position, LOWER_LIMIT, UPPER_LIMIT);
  }
}