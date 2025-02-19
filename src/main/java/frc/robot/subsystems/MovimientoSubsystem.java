package frc.robot.subsystems;

import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.SparkMaxConfig;

import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.DriveConstants;

public class MovimientoSubsystem extends SubsystemBase {
  private final SparkMax MotorMovimientoIzquierdoLider;
  private final SparkMax MotorMovimientoIzquierdoSeguidor;
  private final SparkMax MotorMovimientoDerechoLider;
  private final SparkMax MotorMovimientoDerechoSeguidor;

  public final RelativeEncoder leftEncoder;
  public final RelativeEncoder rightEncoder;

  private final DifferentialDrive drive;

  final double kDriveTickAMetros = (15.24 * Math.PI * 1.0 / 100.0) / 2.1;

  public MovimientoSubsystem() {
    // create brushed motors for drive
    MotorMovimientoIzquierdoLider = new SparkMax(DriveConstants.MotorMovimientoIzquierdoLider_ID, MotorType.kBrushed);
    MotorMovimientoIzquierdoSeguidor = new SparkMax(DriveConstants.MotorMovimientoIzquierdoSeguidor_ID,
        MotorType.kBrushed);
    MotorMovimientoDerechoLider = new SparkMax(DriveConstants.MotorMovimientoDerechoLider_ID, MotorType.kBrushed);
    MotorMovimientoDerechoSeguidor = new SparkMax(DriveConstants.MotorMovimientoDerechoSeguidor_ID, MotorType.kBrushed);

    // set up differential drive class
    drive = new DifferentialDrive(MotorMovimientoIzquierdoLider, MotorMovimientoDerechoLider);
    MotorMovimientoIzquierdoLider.setCANTimeout(DriveConstants.CAN_TIMEOUT);
    MotorMovimientoDerechoLider.setCANTimeout(DriveConstants.CAN_TIMEOUT);
    MotorMovimientoIzquierdoSeguidor.setCANTimeout(DriveConstants.CAN_TIMEOUT);
    MotorMovimientoDerechoSeguidor.setCANTimeout(DriveConstants.CAN_TIMEOUT);

    SparkMaxConfig config = new SparkMaxConfig();
    config.voltageCompensation(12);
    config.smartCurrentLimit(DriveConstants.DRIVE_MOTOR_CURRENT_LIMIT);

    config.follow(MotorMovimientoIzquierdoLider);
    MotorMovimientoIzquierdoSeguidor.configure(config, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
    config.follow(MotorMovimientoDerechoLider);
    MotorMovimientoDerechoSeguidor.configure(config, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);

    // Remove following, then apply config to right leader
    config.disableFollowerMode();
    MotorMovimientoDerechoLider.configure(config, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
    // Set conifg to inverted and then apply to left leader. Set Left side inverted
    // so that postive values drive both sides forward
    config.inverted(true);
    MotorMovimientoIzquierdoLider.configure(config, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);

    this.leftEncoder = MotorMovimientoIzquierdoLider.getEncoder();
    this.rightEncoder = MotorMovimientoDerechoLider.getEncoder();
  }

  @Override
  public void periodic() {
    SmartDashboard.putData(drive);
  }

  // Set drive speeds using DifferentialDrive (tank drive)
  public void setDriveSpeed(double leftSpeed, double rightSpeed) {
    drive.tankDrive(leftSpeed, rightSpeed);
  }

  // Get encoder positions (converted to meters)
  public double getLeftEncoderPosition() {
    return leftEncoder.getPosition() * kDriveTickAMetros;
  }

  public double getRightEncoderPosition() {
    return -(rightEncoder.getPosition() * kDriveTickAMetros);
  }

  // Arcade drive method (single joystick for forward/backward and rotation)
  public void driveArcade(double xSpeed, double zRotation) {
    drive.arcadeDrive(xSpeed, zRotation);
  }
}
