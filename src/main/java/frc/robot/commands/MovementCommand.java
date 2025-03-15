package frc.robot.commands;

import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.DeadZone;
import frc.robot.Constants.DriveConstants;
import frc.robot.subsystems.MovementSubsystem;
import frc.robot.utils.Utils;

import java.util.function.BooleanSupplier;
import java.util.function.DoubleSupplier;

public class MovementCommand extends Command {
  private final DoubleSupplier xSpeed;
  private final DoubleSupplier zRotation;
  private final BooleanSupplier bumper;
  private final MovementSubsystem driveSubsystem;
  private final SlewRateLimiter filter = new SlewRateLimiter(0.5);
  private double lastTime;
  private double accel;

  public MovementCommand(
    DoubleSupplier xSpeed,
    BooleanSupplier bumper,
    DoubleSupplier zRotation,
    MovementSubsystem driveSubsystem
  ) {
    this.xSpeed = xSpeed;
    this.bumper = bumper;
    this.zRotation = zRotation;
    this.driveSubsystem = driveSubsystem;
    this.lastTime = Timer.getFPGATimestamp();

    addRequirements(this.driveSubsystem);
  }

  // Runs each time the command is scheduled.
  @Override
  public void initialize() {
    SmartDashboard.putNumber("Giro", 0);
    SmartDashboard.putNumber("Velocidad", 0);
    SmartDashboard.putNumber("Aceleracion", 0);
    SmartDashboard.putNumber("Giro Total", 0);
    SmartDashboard.putNumber("Velocidad Total", 0);
  }

  // Runs every cycle while the command is scheduled (~50 times per second)
  @Override
  public void execute() {
    double speed = xSpeed.getAsDouble();
    double rotation = zRotation.getAsDouble();

    double deltaTime = Timer.getFPGATimestamp() - lastTime;

    boolean velocityNotAtLimit = Math.abs(speed) < DriveConstants.AXIS_VELOCIDAD_LIMIT;
    boolean giroNotAtLimit = Math.abs(rotation) < DriveConstants.AXIS_GIRO_LIMIT;
    boolean accelerator = bumper.getAsBoolean();
    boolean accelNotAtLimit = this.accel < DriveConstants.BUMPER_ACEL_LIMIT;

    if (velocityNotAtLimit && giroNotAtLimit) {
      this.accel = DriveConstants.DEAD_POINT;
    } else if (accelerator && accelNotAtLimit) {
      this.accel += DriveConstants.ACEL_AUMENTO * deltaTime;
      this.accel = Math.min(this.accel, DriveConstants.BUMPER_ACEL_LIMIT);
    } else if (!accelerator && this.accel > DriveConstants.DEAD_POINT) {
      this.accel -= DriveConstants.ACEL_AUMENTO * deltaTime;
      this.accel = Math.max(this.accel, DriveConstants.DEAD_POINT);
    }

    speed = Utils.applyDeadZone(speed, DeadZone.MOVEMENT);
    rotation = Utils.applyDeadZone(rotation, DeadZone.MOVEMENT);

    updateDashboard(deltaTime, rotation, speed);

    if (speed > DriveConstants.DEAD_POINT) {
      speed = filter.calculate(speed + accel);
    } else if (speed <= DriveConstants.AXIS_VELOCIDAD_LIMIT) {
      speed = filter.calculate(speed - accel);
    }
    this.lastTime = Timer.getFPGATimestamp();

    driveSubsystem.driveArcade(speed, rotation);
  }

  private void updateDashboard(double deltaTime, double giro, double velocidad) {
    SmartDashboard.putNumber("Tiempo", deltaTime);
    SmartDashboard.putNumber("Giro", giro);
    SmartDashboard.putNumber("Velocidad", velocidad);
    SmartDashboard.putNumber("Aceleracion", accel);
    SmartDashboard.putNumber("Giro Total", giro + accel);
    SmartDashboard.putNumber("Velocidad Total", velocidad + accel);
  }

  // Runs each time the command ends via isFinished or being interrupted.
  @Override
  public void end(boolean isInterrupted) {
  }

  // Runs every cycle while the command is scheduled to check if the command is
  // finished
  @Override
  public boolean isFinished() {
    // Return false to indicate that this command never ends. It can be interrupted
    // by another command needing the same subsystem.
    return false;
  }
}