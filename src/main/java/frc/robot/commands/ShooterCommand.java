package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.RodLateralesSubsystem;

public class ShooterCommand extends Command {
  private final double shooterPower;

  private final RodLateralesSubsystem rodLateralesSubsystem;

  public ShooterCommand(RodLateralesSubsystem rodLateralesSubsystem, double shooterPower) {
    this.shooterPower = shooterPower;
    this.rodLateralesSubsystem = rodLateralesSubsystem;

    addRequirements(this.rodLateralesSubsystem);
  }

  @Override
  public void initialize() {
  }

  @Override
  public void execute() {
    rodLateralesSubsystem.shot(shooterPower);
  }

  @Override
  public void end(boolean isInterrupted) {
    rodLateralesSubsystem.shot(0);
  }

  @Override
  public boolean isFinished() {
    return false;
  }
}
