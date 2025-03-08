package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.RodInteriorSubsystem;

public class RodInteriorCommand extends Command {
  private final double power;

  private final RodInteriorSubsystem rodInteriorSubsystem;

  public RodInteriorCommand(RodInteriorSubsystem rodInteriorSubsystem, double power) {
    this.power = power;
    this.rodInteriorSubsystem = rodInteriorSubsystem;

    addRequirements(this.rodInteriorSubsystem);
  }

  @Override
  public void execute() {
    rodInteriorSubsystem.andarRodillo(power);
  }

  @Override
  public void end(boolean isInterrupted) {
    rodInteriorSubsystem.andarRodillo(0);
  }
}
