package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.RodLateralesSubsystem;

public class RodLateralesCommand extends Command {
  private final double powerRodLaterales;

  private final RodLateralesSubsystem rodLateralesSubsystem;

  public RodLateralesCommand(RodLateralesSubsystem rodLateralesSubsystem, double powerRodLaterales) {
    this.powerRodLaterales = powerRodLaterales;
    this.rodLateralesSubsystem = rodLateralesSubsystem;

    addRequirements(this.rodLateralesSubsystem);
  }

  @Override
  public void initialize() {
  }

  @Override
  public void execute() {
    rodLateralesSubsystem.andarRodilloLaterales(powerRodLaterales);
  }

  @Override
  public void end(boolean isInterrupted) {
    rodLateralesSubsystem.andarRodilloLaterales(powerRodLaterales);
  }

  @Override
  public boolean isFinished() {
    return false;
  }
}
