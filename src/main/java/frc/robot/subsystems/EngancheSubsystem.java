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
import frc.robot.Constants.EngancheConstants;
import frc.robot.Constants.NEOMotorsConstants;
import frc.robot.utils.MotorSlowdownLimits;

public class EngancheSubsystem extends SubsystemBase {
  private final SparkMax motor;
  private final RelativeEncoder engancheEncoder;

  private static final double SLOWDOWN_RANGE = 7.0;
  private static final double UPPER_LIMIT = 30.0;
  private static final double LOWER_LIMIT = 0.0;
  public static final String DASH_POS = "Enganche/Posicion";
  public static final String DASH_RESET_ENCODER = "Enganche/Reset Encoder";
  private final MotorSlowdownLimits slowdownLimits;

  public EngancheSubsystem() {
    motor = new SparkMax(EngancheConstants.CAN_ID, MotorType.kBrushless);
    motor.setCANTimeout(DriveConstants.CAN_TIMEOUT);
    motor.configure(getLeaderConfig(), ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
    engancheEncoder = motor.getEncoder();
    engancheEncoder.setPosition(0);
    SmartDashboard.putData(DASH_RESET_ENCODER, new InstantCommand(() -> engancheEncoder.setPosition(0)));
    slowdownLimits = new MotorSlowdownLimits(
      LOWER_LIMIT,
      UPPER_LIMIT,
      SLOWDOWN_RANGE,
      this::getEnganchePosition,
      motor::set
    );
  }

  private static SparkMaxConfig getLeaderConfig() {
    SparkMaxConfig leaderConfig = new SparkMaxConfig();

    leaderConfig.softLimit
      .forwardSoftLimitEnabled(true)
      .forwardSoftLimit(30)
      .reverseSoftLimitEnabled(true)
      .reverseSoftLimit(0);

    leaderConfig
      .voltageCompensation(NEOMotorsConstants.VOLTAGE_COMPENSATION)
      .smartCurrentLimit(NEOMotorsConstants.CURRENT_LIMIT);

    return leaderConfig;
  }

  @Override
  public void periodic() {
    SmartDashboard.putNumber(DASH_POS, getEnganchePosition());
  }

  public double getEnganchePosition() {
    return engancheEncoder.getPosition();
  }

  public void moveEnganche(double speed) {
    slowdownLimits.move(speed);
  }
}