package frc.robot.subsystems;

import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.SparkBase;
import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.spark.SparkLowLevel;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.ClosedLoopConfig;
import com.revrobotics.spark.config.SparkBaseConfig;
import com.revrobotics.spark.config.SparkMaxConfig;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public abstract class SingleMotorVelocitySubsystem extends SubsystemBase implements MovableSubsystem {
  protected final RelativeEncoder encoder;
  protected final SparkClosedLoopController controller;
  private double targetVelocity = 0;

  public SingleMotorVelocitySubsystem() {
    SparkMax motor = new SparkMax(getCanId(), SparkLowLevel.MotorType.kBrushless);
    motor.configure(getMotorConfig(), SparkBase.ResetMode.kResetSafeParameters, SparkBase.PersistMode.kPersistParameters);
    encoder = motor.getEncoder();
    controller = motor.getClosedLoopController();
  }

  protected abstract int getCanId();

  protected SparkMaxConfig getMotorConfig() {
    SparkMaxConfig motorConfig = new SparkMaxConfig();
    motorConfig.closedLoop
      .p(0.02)
      .i(0.00004)
      .iZone(1.7)
      .d(0)
      .feedbackSensor(ClosedLoopConfig.FeedbackSensor.kPrimaryEncoder)
      .maxMotion
      .maxVelocity(10000)
      .allowedClosedLoopError(0.25)
      .maxAcceleration(8000);
    motorConfig
      .voltageCompensation(Constants.NEOMotorsConstants.VOLTAGE_COMPENSATION)
      .smartCurrentLimit(Constants.NEOMotorsConstants.CURRENT_LIMIT)
      .idleMode(SparkBaseConfig.IdleMode.kBrake);
    return motorConfig;
  }

  @Override
  public void periodic() {
    controller.setReference(this.targetVelocity, SparkBase.ControlType.kMAXMotionVelocityControl);
  }

  @Override
  public void reset() {
    encoder.setPosition(0);
    controller.setReference(0, SparkBase.ControlType.kMAXMotionVelocityControl);
  }

  @Override
  public double getActual() {
    return encoder.getVelocity();
  }

  @Override
  public double getTarget() {
    return targetVelocity;
  }

  @Override
  public void move(double target) {
    this.targetVelocity = target;
  }
}
