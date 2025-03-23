package frc.robot.commands;

import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.RodLateralesSubsystem;

public class SideRodCommand extends Command {
  private final double power;

  private final RodLateralesSubsystem rodLateralesSubsystem;
  private final SlewRateLimiter limiter = new SlewRateLimiter(0.5);

  public SideRodCommand(RodLateralesSubsystem rodLateralesSubsystem, double power) {
    this.power = power;
    this.rodLateralesSubsystem = rodLateralesSubsystem;

    addRequirements(this.rodLateralesSubsystem);
  }

  @Override
  public void execute() {
    rodLateralesSubsystem.andarRodillo(limiter.calculate(power));
  }

  @Override
  public void end(boolean isInterrupted) {
    rodLateralesSubsystem.andarRodillo(0);
  }
}
