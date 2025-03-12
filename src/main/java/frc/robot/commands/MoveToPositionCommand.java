package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.subsystems.MovableSubsystem;
import frc.robot.utils.Utils;

import java.util.function.DoubleSupplier;

public class MoveToPositionCommand extends Command {
  protected final MovableSubsystem subsystem;
  double targetPosition;
  double tolerance;

  public MoveToPositionCommand(double targetPosition, MovableSubsystem subsystem, double tolerance) {
    this.targetPosition = targetPosition;
    this.subsystem = subsystem;
    this.tolerance = tolerance;
  }

  @Override
  public void execute() {
    subsystem.move(targetPosition);
  }

  @Override
  public boolean isFinished() {
    return Math.abs(subsystem.getActualPosition() - targetPosition) < tolerance;
  }
}
