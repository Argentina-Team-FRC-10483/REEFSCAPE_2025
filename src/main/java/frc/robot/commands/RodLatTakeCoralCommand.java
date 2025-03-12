package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.RodLateralesSubsystem;

public class RodLatTakeCoralCommand extends Command {
  private final double power;
  private final double duration;
  private final RodLateralesSubsystem rodLateralesSubsystem;
  private long startTime;

  public RodLatTakeCoralCommand(RodLateralesSubsystem rodLateralesSubsystem, double power, double duration) {
    this.power = power;
    this.duration = duration;
    this.rodLateralesSubsystem = rodLateralesSubsystem;

    addRequirements(this.rodLateralesSubsystem);
  }

  @Override
  public void initialize() {
    startTime = System.currentTimeMillis();
    rodLateralesSubsystem.andarRodillo(power);
  }

  @Override
  public void end(boolean isInterrupted) {
    rodLateralesSubsystem.andarRodillo(0);
  }

  @Override
  public boolean isFinished() {
    return System.currentTimeMillis() - startTime >= duration * 1000;
  }
}