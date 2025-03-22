package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.RodLateralesSubsystem;

public class SideRodCommandTimed extends Command {
  private final double power;
  private final double durationSeconds;
  private final RodLateralesSubsystem rodLateralesSubsystem;
  private double startTime;

  public SideRodCommandTimed(RodLateralesSubsystem rodLateralesSubsystem, double power, double durationSeconds) {
    this.power = power;
    this.rodLateralesSubsystem = rodLateralesSubsystem;
    this.durationSeconds = durationSeconds;

    addRequirements(this.rodLateralesSubsystem);
  }

  @Override
  public void initialize() {
      startTime = System.currentTimeMillis();
  }

  @Override
  public void execute() {
    rodLateralesSubsystem.andarRodillo(power);
  }

  @Override
  public boolean isFinished() {
      return System.currentTimeMillis() - startTime > durationSeconds * 1000;
  }

  @Override
  public void end(boolean isInterrupted) {
    rodLateralesSubsystem.andarRodillo(0);
  }
}
