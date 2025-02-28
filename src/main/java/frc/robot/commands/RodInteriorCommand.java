package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.RodInteriorSubsystem;

public class RodInteriorCommand extends Command {
  private final double powerRodInterior;

  private final RodInteriorSubsystem rodInteriorSubsystem;

  public RodInteriorCommand(RodInteriorSubsystem rodInteriorSubsystem, double powerRodInterior) {
    this.powerRodInterior = powerRodInterior;
    this.rodInteriorSubsystem = rodInteriorSubsystem;

    addRequirements(this.rodInteriorSubsystem);
  }

  @Override
  public void initialize() {
  }

  @Override
  public void execute() {
    rodInteriorSubsystem.andarRodilloInterior(powerRodInterior);
  }

  @Override
  public void end(boolean isInterrupted) {
    rodInteriorSubsystem.andarRodilloInterior(powerRodInterior);
  }

  @Override
  public boolean isFinished() {
    return false;
  }
}
