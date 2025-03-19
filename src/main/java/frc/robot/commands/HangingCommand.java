package frc.robot.commands;

import java.util.function.DoubleSupplier;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.HangingSubsystem;

public class HangingCommand extends Command {
  private final DoubleSupplier hangingPower;
  private final HangingSubsystem hangingSubsystem;

  public HangingCommand(HangingSubsystem hangingSubsystem, DoubleSupplier hangingPower) {
    this.hangingPower = hangingPower;
    this.hangingSubsystem = hangingSubsystem;

    addRequirements(this.hangingSubsystem);
  }

  @Override
  public void execute() {
    hangingSubsystem.move(hangingPower.getAsDouble());
  }

  @Override
  public void end(boolean isInterrupted) {
    hangingSubsystem.move(0);
  }
}
