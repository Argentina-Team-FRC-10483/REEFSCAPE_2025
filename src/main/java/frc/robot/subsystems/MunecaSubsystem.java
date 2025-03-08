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
import frc.robot.Constants.MunecaConstants;
import frc.robot.Constants.NEOMotorsConstants;
import frc.robot.utils.MotorSlowdownLimits;

public class MunecaSubsystem extends SubsystemBase {
  private final SparkMax motor;
  private final RelativeEncoder munecaEncoder;
  private final MotorSlowdownLimits slowdownLimits;

  public static final double SLOWDOWN_RANGE = 7.0;
  private static final double UPPER_LIMIT = 0.0;
  private static final double LOWER_LIMIT = -15.0;
  public static final String DASH_MUNECA_POS = "Muñeca Posicion";
  public static final String DASH_RESET_MUNECA_ENCODER = "Reiniciar Encoder Muñeca";

  public MunecaSubsystem() {
    motor = new SparkMax(MunecaConstants.MUNECA_MOTOR_ID, MotorType.kBrushless);
    motor.setCANTimeout(DriveConstants.CAN_TIMEOUT);
    motor.configure(getLeaderConfig(), ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
    munecaEncoder = motor.getEncoder();
    munecaEncoder.setPosition(0);
    SmartDashboard.putData(DASH_RESET_MUNECA_ENCODER, new InstantCommand(() -> munecaEncoder.setPosition(0)));
    slowdownLimits = new MotorSlowdownLimits(
      LOWER_LIMIT,
      UPPER_LIMIT,
      SLOWDOWN_RANGE,
      this::getMunecaPosition,
      motor::set
    );
  }

  private static SparkMaxConfig getLeaderConfig() {
    SparkMaxConfig leaderConfig = new SparkMaxConfig();

    leaderConfig.softLimit
      .forwardSoftLimitEnabled(true)
      .forwardSoftLimit(0)
      .reverseSoftLimitEnabled(true)
      .reverseSoftLimit(-15);

    leaderConfig
      .voltageCompensation(NEOMotorsConstants.VOLTAGE_COMPENSATION_NEO)
      .smartCurrentLimit(NEOMotorsConstants.CURRENT_LIMIT_NEO)
      .idleMode(SparkBaseConfig.IdleMode.kBrake);

    return leaderConfig;
  }

  @Override
  public void periodic() {
    SmartDashboard.putNumber(DASH_MUNECA_POS, getMunecaPosition());
  }

  public double getMunecaPosition() {
    return munecaEncoder.getPosition();
  }

  public void moveMuneca(double speed) {
    slowdownLimits.move(speed);
  }
}