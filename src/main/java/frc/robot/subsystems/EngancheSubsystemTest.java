package frc.robot.subsystems;

import com.revrobotics.spark.SparkBase;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.SparkBaseConfig;
import com.revrobotics.spark.config.SparkMaxConfig;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.DriveConstants;
import frc.robot.Constants.MunecaConstants;
import frc.robot.Constants.NEOMotorsConstants;

public class EngancheSubsystemTest extends SubsystemBase implements MovableSubsystem {
  private final SparkMax motor = new SparkMax(MunecaConstants.MUNECA_MOTOR_ID, MotorType.kBrushless);
  private final SparkClosedLoopController controller = motor.getClosedLoopController();
  private final RelativeEncoder engancheEncoderTest;
  private static final double UPPER_LIMIT = 0;
  private static final double LOWER_LIMIT = -22;
  public static final String DASH_ENGANCHE_POS = "Enganche position";
  public static final String DASH_RESET_ENGANCHE_ENCODER = "Reiniciar Encoder Enganche";
  private double position;

  public EngancheSubsystemTest() {
    motor.setCANTimeout(DriveConstants.CAN_TIMEOUT);
    motor.configure(getLeaderConfig(), ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
    engancheEncoderTest = motor.getEncoder();
    reset();
    SmartDashboard.putData(DASH_RESET_ENGANCHE_ENCODER, new InstantCommand(this::reset));
  }

  private static SparkMaxConfig getLeaderConfig() {
    SparkMaxConfig leaderConfig = new SparkMaxConfig();

    leaderConfig.closedLoop
      .p(0.02)
      .iZone(1.7)
      .i(0.00004) // 0.001
      .d(0)
      // .velocityFF(0)hhhhhhhhhhhhhhhhhhh
      .maxMotion
      .maxVelocity(0.4)
      .allowedClosedLoopError(1)
      .maxAcceleration(0.05);

    leaderConfig
      .voltageCompensation(NEOMotorsConstants.VOLTAGE_COMPENSATION_NEO)
      .smartCurrentLimit(NEOMotorsConstants.CURRENT_LIMIT_NEO)
      .idleMode(SparkBaseConfig.IdleMode.kBrake);

    return leaderConfig;
  }

  @Override
  public void reset(){
    engancheEncoderTest.setPosition(0);
    position = 0;
    controller.setReference(this.position, SparkBase.ControlType.kPosition);
  }

  @Override
  public void periodic() {
    SmartDashboard.putNumber(DASH_ENGANCHE_POS, getActualPosition());
    SmartDashboard.putNumber("Target Enganche pos", position);
    controller.setReference(this.position, SparkBase.ControlType.kPosition);
  }

  @Override
  public double getActualPosition() {
    return engancheEncoderTest.getPosition();
  }

  @Override
  public double getTargetPosition() {
    return position;
  }

  @Override
  public void move(double position) {
    this.position = MathUtil.clamp(position, LOWER_LIMIT, UPPER_LIMIT);
  }
}