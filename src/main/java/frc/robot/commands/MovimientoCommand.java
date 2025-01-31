package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.MovimientoSubsystem;
import java.util.function.DoubleSupplier;

public class MovimientoCommand extends Command {
  private final DoubleSupplier xSpeed;
  private final DoubleSupplier zRotation;
  private final MovimientoSubsystem driveSubsystem;

  public MovimientoCommand(
      DoubleSupplier xSpeed, DoubleSupplier zRotation, MovimientoSubsystem driveSubsystem) {
    this.xSpeed = xSpeed;
    this.zRotation = zRotation;
    this.driveSubsystem = driveSubsystem;

    addRequirements(this.driveSubsystem);
  }

  // Runs each time the command is scheduled.
  @Override
  public void initialize() {
  }

  // Runs every cycle while the command is scheduled (~50 times per second)
  @Override
  public void execute() {
    driveSubsystem.driveArcade(xSpeed.getAsDouble()*0.6, zRotation.getAsDouble()*0.6);
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
