package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.subsystems.MovableSubsystem;
import frc.robot.utils.Utils;

import java.util.function.DoubleSupplier;

public class IncrementalMoveCommand extends Command {
  protected final DoubleSupplier positionChanger;
  protected final MovableSubsystem subsystem;

  public IncrementalMoveCommand(DoubleSupplier positionChanger, MovableSubsystem subsystem) {
    this.positionChanger = positionChanger;
    this.subsystem = subsystem;
  }

  @Override
  public void execute() {
    double targetPosition = subsystem.getTargetPosition();
    // TODO: Make constant generic yay
    targetPosition += Utils.applyDeadZone(positionChanger.getAsDouble(), Constants.DeadZone.ELEVATOR);
    subsystem.move(targetPosition);
  }
}