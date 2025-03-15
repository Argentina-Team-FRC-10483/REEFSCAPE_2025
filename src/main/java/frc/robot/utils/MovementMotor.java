package frc.robot.utils;

import com.revrobotics.RelativeEncoder;
import com.revrobotics.sim.SparkRelativeEncoderSim;
import com.revrobotics.spark.SparkBase;
import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.spark.SparkLowLevel;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.ClosedLoopConfig;
import com.revrobotics.spark.config.SparkMaxConfig;
import edu.wpi.first.networktables.DoublePublisher;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

import static com.revrobotics.spark.SparkBase.ControlType.kMAXMotionVelocityControl;
import static com.revrobotics.spark.SparkBase.ControlType.kVelocity;

public class MovementMotor extends SubsystemBase {
  static final double kDriveRotToMts = (15.24 * Math.PI * 1.0 / 100.0) / 2.1;
  static final double kDriveRPMToMps = (2 * Math.PI * 0.0762) / 60;

  public final SparkMax leader;
  public final SparkMax follow;
  public final RelativeEncoder encoder;
  public final SparkClosedLoopController controller;
  public final SparkRelativeEncoderSim encoderSim;
  public double targetVelocity;
  private final DoublePublisher positionPublisher, velocityPublisher, targetVelocityPublisher;

  public MovementMotor(int leaderId, int followId, boolean inverted, String name){
    positionPublisher = NetworkTableInstance.getDefault().getTable("Motor/" + name).getDoubleTopic("position").publish();
    velocityPublisher = NetworkTableInstance.getDefault().getTable("Motor/" + name).getDoubleTopic("velocity").publish();
    targetVelocityPublisher = NetworkTableInstance.getDefault().getTable("Motor/" + name).getDoubleTopic("targetVelocity").publish();
    leader = new SparkMax(leaderId, SparkLowLevel.MotorType.kBrushless);
    follow = new SparkMax(followId, SparkLowLevel.MotorType.kBrushless);
    controller = leader.getClosedLoopController();
    encoder = leader.getEncoder();
    encoderSim = new SparkRelativeEncoderSim(leader);
    leader.setCANTimeout(Constants.DriveConstants.CAN_TIMEOUT);
    follow.setCANTimeout(Constants.DriveConstants.CAN_TIMEOUT);
    targetVelocity = 0;
    configureMotors(inverted);
  }

  private void configureMotors(boolean inverted) {
    SparkMaxConfig config = new SparkMaxConfig();
    config.voltageCompensation(12);
    config.smartCurrentLimit(Constants.DriveConstants.DRIVE_MOTOR_CURRENT_LIMIT);
    config.closedLoop.
      p(0.3)
      .i(0)
      .d(0)
      .feedbackSensor(ClosedLoopConfig.FeedbackSensor.kPrimaryEncoder);
    config.closedLoop.maxMotion
      .maxAcceleration(0.01)
      .maxVelocity(0.05);
    config.alternateEncoder
      .positionConversionFactor(kDriveRotToMts)
      .velocityConversionFactor(kDriveRPMToMps);
    config.follow(leader);
    follow.configure(config, SparkBase.ResetMode.kResetSafeParameters, SparkBase.PersistMode.kPersistParameters);
    config.disableFollowerMode();
    config.inverted(inverted);
    leader.configure(config, SparkBase.ResetMode.kResetSafeParameters, SparkBase.PersistMode.kPersistParameters);
  }

  @Override
  public void periodic(){
//    controller.setReference(this.targetVelocity, kMAXMotionVelocityControl);
    leader.set(targetVelocity);
    positionPublisher.accept(encoder.getPosition());
    velocityPublisher.accept(encoder.getVelocity());
    targetVelocityPublisher.accept(targetVelocity);
  }

  public void set(double speed){
    targetVelocity = speed;
  }

  public void updateSimulation(double positionMeters, double velocityMPerS) {
    encoderSim.setPosition(positionMeters / kDriveRotToMts);
    encoderSim.setVelocity(velocityMPerS / kDriveRotToMts);
  }
}
