package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.EngancheSubsystem;

public class EngancheCommand extends Command {
  private final double power;

  private final EngancheSubsystem engancheSubsystem;

  public EngancheCommand(EngancheSubsystem engancheSubsystem, double power) {
    this.power = power;
    this.engancheSubsystem = engancheSubsystem;

    addRequirements(this.engancheSubsystem);
  }

  @Override
  public void initialize() {
  }

  @Override
  public void execute() {
    engancheSubsystem.andarRodillo(power);
  }

  @Override
  public void end(boolean isInterrupted) {
    engancheSubsystem.andarRodillo(0);
  }

  @Override
  public boolean isFinished() {
    return false;
  }
}
