package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.RodInteriorSubsystem;

public class TimedRollerScoreCommand extends Command {
  private final double power;
  private final double duration;
  private final RodInteriorSubsystem rodInteriorSubsystem;
  private long startTime;

  public TimedRollerScoreCommand(RodInteriorSubsystem rodInteriorSubsystem, double power, double duration) {
    this.power = power;
    this.duration = duration;
    this.rodInteriorSubsystem = rodInteriorSubsystem;

    addRequirements(this.rodInteriorSubsystem);
  }

  @Override
  public void initialize() {
    startTime = System.currentTimeMillis();
    rodInteriorSubsystem.andarRodillo(power);
  }

  @Override
  public void execute() {
  }

  @Override
  public void end(boolean isInterrupted) {
    rodInteriorSubsystem.andarRodillo(0);
  }

  @Override
  public boolean isFinished() {
    return System.currentTimeMillis() - startTime >= duration * 1000;
  }
}