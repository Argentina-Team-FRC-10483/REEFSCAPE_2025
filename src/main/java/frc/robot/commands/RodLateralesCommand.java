package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.RodLateralesSubsystem;

public class RodLateralesCommand extends Command {
  private final double power;

  private final RodLateralesSubsystem rodLateralesSubsystem;

  public RodLateralesCommand(RodLateralesSubsystem rodLateralesSubsystem, double power) {
    this.power = power;
    this.rodLateralesSubsystem = rodLateralesSubsystem;

    addRequirements(this.rodLateralesSubsystem);
  }

  @Override
  public void initialize() {
  }

  @Override
  public void execute() {
    rodLateralesSubsystem.andarRodillo(power);
  }

  @Override
  public void end(boolean isInterrupted) {
    rodLateralesSubsystem.andarRodillo(0);
  }

  @Override
  public boolean isFinished() {
    return false;
  }
}
