package frc.robot.commands;

import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.wpilibj.Timer;
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
  private final SlewRateLimiter filter = new SlewRateLimiter(1.5);
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

  // Runs every cycle while the command is scheduled (~50 times per second)
  @Override
  public void execute() {
    double speed = xSpeed.getAsDouble();
    double rotation = zRotation.getAsDouble();

    double deltaTime = Timer.getFPGATimestamp() - lastTime;

    boolean velocityNotAtLimit = Math.abs(speed) < DriveConstants.AXIS_SPEED_LIMIT;
    boolean giroNotAtLimit = Math.abs(rotation) < DriveConstants.AXIS_TURN_LIMIT;
    boolean accelerator = bumper.getAsBoolean();
    boolean accelNotAtLimit = this.accel < DriveConstants.BUMPER_ACCEL_LIMIT;

    if (velocityNotAtLimit && giroNotAtLimit) {
      this.accel = DriveConstants.DEAD_POINT;
    } else if (accelerator && accelNotAtLimit) {
      this.accel += DriveConstants.ACCEL_INCREASE * deltaTime;
      this.accel = Math.min(this.accel, DriveConstants.BUMPER_ACCEL_LIMIT);
    } else if (!accelerator && this.accel > DriveConstants.DEAD_POINT) {
      this.accel -= DriveConstants.ACCEL_INCREASE * deltaTime;
      this.accel = Math.max(this.accel, DriveConstants.DEAD_POINT);
    }

    speed = Utils.applyDeadZone(speed, DeadZone.MOVEMENT);
    rotation = Utils.applyDeadZone(rotation, DeadZone.MOVEMENT);

    if (speed > DriveConstants.DEAD_POINT) {
      speed = filter.calculate(speed + accel);
    } else if (speed <= DriveConstants.AXIS_SPEED_LIMIT) {
      speed = filter.calculate(speed - accel);
    }
    this.lastTime = Timer.getFPGATimestamp();
    driveSubsystem.driveArcade(speed, rotation);
  }
}