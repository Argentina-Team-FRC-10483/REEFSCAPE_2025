package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.MunecaSubsystem;

public class MunecaToPositionCommand extends Command {
  private final MunecaSubsystem munecaSubsystem;
  private final double targetPosition;

  public MunecaToPositionCommand(MunecaSubsystem munecaSubsystem, double targetPosition) {
    this.munecaSubsystem = munecaSubsystem;
    this.targetPosition = targetPosition;
    addRequirements(this.munecaSubsystem);
  }

  @Override
  public void execute() {
    double currentPosition = munecaSubsystem.getMunecaPosition();
    double speed = 0.3;

    if (currentPosition < targetPosition) {
      double distanceRemaining = targetPosition - currentPosition;

      if (distanceRemaining <= MunecaSubsystem.SLOWDOWN_RANGE) {
        double slowdownFactor = distanceRemaining / MunecaSubsystem.SLOWDOWN_RANGE;
        speed *= Math.max(slowdownFactor, 0.05);
      }
    } else if (currentPosition > targetPosition) {
      double distanceRemaining = currentPosition - targetPosition;

      if (distanceRemaining <= MunecaSubsystem.SLOWDOWN_RANGE) {
        double slowdownFactor = distanceRemaining / MunecaSubsystem.SLOWDOWN_RANGE;
        speed *= Math.max(slowdownFactor, 0.05);
      }
      speed = -speed;

    }

    if (Math.abs(speed) < 0.05) {
      speed = Math.signum(speed) * 0.05;
    }

    munecaSubsystem.moveMuneca(speed);
  }

  @Override
  public void end(boolean interrupted) {
    munecaSubsystem.moveMuneca(0);
  }

  @Override
  public boolean isFinished() {
    return munecaSubsystem.getMunecaPosition() >= (targetPosition - 0.5) && munecaSubsystem.getMunecaPosition() <= (targetPosition + 0.5);
  }
}