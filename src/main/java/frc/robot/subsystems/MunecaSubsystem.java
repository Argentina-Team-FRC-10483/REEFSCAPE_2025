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

public class MunecaSubsystem extends SubsystemBase implements MovableSubsystem {
  private final SparkMax motor = new SparkMax(MunecaConstants.MUNECA_MOTOR_ID, MotorType.kBrushless);
  private final SparkClosedLoopController controller = motor.getClosedLoopController();
  private final RelativeEncoder munecaEncoder;
  private static final double UPPER_LIMIT = 2;
  private static final double LOWER_LIMIT = -20;
  public static final String DASH_MUNECA_POS = "Muneca Posicion";
  public static final String DASH_RESET_MUNECA_ENCODER = "Reiniciar Encoder Muñeca";
  private double position;

  public MunecaSubsystem() {
    motor.setCANTimeout(DriveConstants.CAN_TIMEOUT);
    motor.configure(getLeaderConfig(), ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
    munecaEncoder = motor.getEncoder();
    reset();
    SmartDashboard.putData(DASH_RESET_MUNECA_ENCODER, new InstantCommand(this::reset));
  }

  private static SparkMaxConfig getLeaderConfig() {
    SparkMaxConfig leaderConfig = new SparkMaxConfig();

    leaderConfig.closedLoop
      .p(0.02)
      // .iZone(0.5)
      .i(0)
      .d(0)
      // .velocityFF(0)
      .maxMotion
      .maxVelocity(0.4)
      .maxAcceleration(0.05);

    leaderConfig
      .voltageCompensation(NEOMotorsConstants.VOLTAGE_COMPENSATION_NEO)
      .smartCurrentLimit(NEOMotorsConstants.CURRENT_LIMIT_NEO)
      .idleMode(SparkBaseConfig.IdleMode.kBrake);

    return leaderConfig;
  }

  @Override
  public void reset(){
    munecaEncoder.setPosition(0);
    position = 0;
    controller.setReference(this.position, SparkBase.ControlType.kPosition);
  }

  @Override
  public void periodic() {
    SmartDashboard.putNumber(DASH_MUNECA_POS, getActualPosition());
    SmartDashboard.putNumber("Targe Muneca pos", position);
    controller.setReference(this.position, SparkBase.ControlType.kPosition);
  }

  @Override
  public double getActualPosition() {
    return munecaEncoder.getPosition();
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