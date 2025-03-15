package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.MovableSubsystem;

public class MoveToTargetCommand extends Command {
  protected final MovableSubsystem subsystem;
  double target;
  double tolerance;
  private final boolean waitForCompletion;

  public MoveToTargetCommand(double target, MovableSubsystem subsystem) {
    this(target, subsystem, 0.01, false);
  }

  public MoveToTargetCommand(double target, MovableSubsystem subsystem, double tolerance, boolean waitForCompletion) {
    this.target = target;
    this.subsystem = subsystem;
    this.tolerance = tolerance;
    this.waitForCompletion = waitForCompletion;
  }

  @Override
  public void execute() {
    subsystem.move(target);
  }

  @Override
  public boolean isFinished() {
    return !waitForCompletion || Math.abs(subsystem.getActual() - target) < tolerance;
  }
}