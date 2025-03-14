
package frc.robot.subsystems;

import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.SparkMaxConfig;

import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.DriveConstants;

public class MovementSubsystem extends SubsystemBase {
  private final SparkMax leftLeader;
  private final SparkMax leftFollow;
  private final SparkMax rightLeader;
  private final SparkMax rightFollow;

  private final DifferentialDrive drive;

  public MovementSubsystem() {
    // Create brushed motors for drive
    leftLeader = new SparkMax(DriveConstants.LEFT_MOVEMENT_LEADER_MOTOR_ID, MotorType.kBrushed);
    leftFollow = new SparkMax(DriveConstants.LEFT_MOVEMENT_FOLLOW_MOTOR_ID, MotorType.kBrushed);
    rightLeader = new SparkMax(DriveConstants.RIGHT_MOVEMENT_LEADER_MOTOR_ID, MotorType.kBrushed);
    rightFollow = new SparkMax(DriveConstants.RIGHT_MOVEMENT_FOLLOW_MOTOR_ID, MotorType.kBrushed);

    // Set up differential drive class
    drive = new DifferentialDrive(leftLeader, rightLeader);
    leftLeader.setCANTimeout(DriveConstants.CAN_TIMEOUT);
    rightLeader.setCANTimeout(DriveConstants.CAN_TIMEOUT);
    leftFollow.setCANTimeout(DriveConstants.CAN_TIMEOUT);
    rightFollow.setCANTimeout(DriveConstants.CAN_TIMEOUT);

    configureMotors();
  }

  private void configureMotors() {
    SparkMaxConfig config = new SparkMaxConfig();
    config.voltageCompensation(12);
    config.smartCurrentLimit(DriveConstants.DRIVE_MOTOR_CURRENT_LIMIT);

    config.follow(leftLeader);
    leftFollow.configure(config, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
    config.follow(rightLeader);
    rightFollow.configure(config, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);

    // Remove following, then apply config to right leader
    config.disableFollowerMode();
    rightLeader.configure(config, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
    // Set conifg to inverted and then apply to left leader.
    // Set Left side inverted so that positive values drive both sides forward
    config.inverted(true);
    leftLeader.configure(config, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
  }

  @Override
  public void periodic() {
    SmartDashboard.putData(drive);
  }

  // sets the speed of the drive motors
  public void driveArcade(double xSpeed, double zRotation) {
    drive.arcadeDrive(xSpeed, zRotation, false);
  }

  public void curvatureArcade(double xSpeed, double zRotation) {
    drive.curvatureDrive(xSpeed, zRotation, false);
  }
}