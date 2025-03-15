package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.MovableSubsystem;

public class MoveToPositionCommand extends Command {
  protected final MovableSubsystem subsystem;
  double targetPosition;
  double tolerance;
  private final boolean waitForCompletion;

  public MoveToPositionCommand(double targetPosition, MovableSubsystem subsystem, double tolerance, boolean waitForCompletion) {
    this.targetPosition = targetPosition;
    this.subsystem = subsystem;
    this.tolerance = tolerance;
    this.waitForCompletion = waitForCompletion;
  }

  @Override
  public void execute() {
    subsystem.move(targetPosition);
  }

  @Override
  public boolean isFinished() {
    return !waitForCompletion || Math.abs(subsystem.getActualPosition() - targetPosition) < tolerance;
  }
}
