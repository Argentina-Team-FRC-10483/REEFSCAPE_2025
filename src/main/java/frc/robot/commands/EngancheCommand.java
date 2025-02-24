package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.EngancheSubsystem;

public class EngancheCommand extends Command {
  private final double vel;
  private final EngancheSubsystem engancheSubsystem;

  public EngancheCommand(EngancheSubsystem engancheSubsystem, double vel) {
    this.vel = vel;
    this.engancheSubsystem = engancheSubsystem;

    addRequirements(engancheSubsystem);
  }

  @Override
  public void initialize() {

  }

  @Override
  public void execute() {
    engancheSubsystem.enganchar(vel);
  }

  @Override
  public boolean isFinished() {
    return false;
  }

  @Override
  public void end(boolean interrupted) {

  }
}
