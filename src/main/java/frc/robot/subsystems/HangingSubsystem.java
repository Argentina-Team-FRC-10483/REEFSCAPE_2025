package frc.robot.subsystems;

import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.SparkMaxConfig;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.DriveConstants;
import frc.robot.Constants.HangingConstants;
import frc.robot.Constants.NEOMotorsConstants;

public class HangingSubsystem extends SubsystemBase {
  private final SparkMax motor;
  private final RelativeEncoder hangingEncoder;

  private static final double SLOWDOWN_RANGE = 7.0;
  private static final double UPPER_LIMIT = 37.0;
  private static final double LOWER_LIMIT = -48.0;
  public static final String DASH_POS = "Hanging/Pos";
  public static final String DASH_RESET_ENCODER = "Hanging/Reset Encoder";

  public HangingSubsystem() {
    motor = new SparkMax(HangingConstants.CAN_ID, MotorType.kBrushless);

    motor.setCANTimeout(DriveConstants.CAN_TIMEOUT);

    motor.configure(getLeaderConfig(), ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);

    hangingEncoder = motor.getEncoder();
    hangingEncoder.setPosition(0);

    SmartDashboard.putData(DASH_RESET_ENCODER, new InstantCommand(() -> hangingEncoder.setPosition(0)));
  }

  private static SparkMaxConfig getLeaderConfig() {
    SparkMaxConfig leaderConfig = new SparkMaxConfig();

    leaderConfig.softLimit
      .forwardSoftLimitEnabled(true)
      .forwardSoftLimit(UPPER_LIMIT)
      .reverseSoftLimitEnabled(true)
      .reverseSoftLimit(LOWER_LIMIT);

    leaderConfig
      .voltageCompensation(NEOMotorsConstants.VOLTAGE_COMPENSATION)
      .smartCurrentLimit(NEOMotorsConstants.CURRENT_LIMIT);

    return leaderConfig;
  }

  @Override
  public void periodic() {
    SmartDashboard.putNumber(DASH_POS, getHangingPosition());
  }

  public double getHangingPosition() {
    return hangingEncoder.getPosition();
  }

  public void move(double speed) {
    double currentPosition = getHangingPosition();

    boolean inLowerSlowdownZone = currentPosition <= LOWER_LIMIT + SLOWDOWN_RANGE;
    boolean inUpperSlowdownZone = currentPosition >= UPPER_LIMIT - SLOWDOWN_RANGE;
    if (speed < 0 && inLowerSlowdownZone) speed = speed * getLowerSlowdownFactor(currentPosition);
    if (speed > 0 && inUpperSlowdownZone) speed = speed * getUpperSlowdownFactor(currentPosition);
    SmartDashboard.putNumber("velocidad enganche", speed);
    motor.set(speed);
  }

  public double getUpperSlowdownFactor(double currentPosition) {
    return Math.max((UPPER_LIMIT - currentPosition) / SLOWDOWN_RANGE, 0);
  }

  public double getLowerSlowdownFactor(double currentPosition) {
    return Math.max((currentPosition - LOWER_LIMIT) / SLOWDOWN_RANGE, 0);
  }

}