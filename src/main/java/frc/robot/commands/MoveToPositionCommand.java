package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.subsystems.MovableSubsystem;
import frc.robot.utils.Utils;

import java.util.function.DoubleSupplier;

public class MoveToPositionCommand extends Command {
  protected final MovableSubsystem subsystem;
  double targetPosition;

  public MoveToPositionCommand(double targetPosition, MovableSubsystem subsystem) {
    this.targetPosition = targetPosition;
    this.subsystem = subsystem;
  }

  @Override
  public void execute() {
    subsystem.move(targetPosition);
  }

  @Override
  public boolean isFinished() {
    return Math.abs(subsystem.getActualPosition() - targetPosition) < 0.1;
  }
}
