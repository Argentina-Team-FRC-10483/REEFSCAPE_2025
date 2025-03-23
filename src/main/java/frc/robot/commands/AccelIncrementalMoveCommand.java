package frc.robot.commands;

import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.subsystems.MovableSubsystem;
import frc.robot.utils.Utils;

import java.util.function.DoubleSupplier;

public class AccelIncrementalMoveCommand extends Command {
  protected final DoubleSupplier positionChanger;
  protected final MovableSubsystem subsystem;
  private final SlewRateLimiter filter;
  public AccelIncrementalMoveCommand(DoubleSupplier positionChanger, MovableSubsystem subsystem, double speed) {
    this.positionChanger = positionChanger;
    this.subsystem = subsystem;
    this.filter = new SlewRateLimiter(speed);
  }

  @Override
  public void execute() {
    double targetPosition = subsystem.getTargetPosition();
    // TODO: Make constant generic yay
    targetPosition += filter.calculate(Utils.applyDeadZone(positionChanger.getAsDouble(), Constants.DeadZone.ELEVATOR));
    subsystem.move(targetPosition);
  }
}