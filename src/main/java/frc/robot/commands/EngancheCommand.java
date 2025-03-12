package frc.robot.commands;

import java.util.function.DoubleSupplier;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.DeadZone;
import frc.robot.utils.Utils;
import frc.robot.subsystems.EngancheSubsystem;

public class EngancheCommand extends Command {
  private final DoubleSupplier enganchePower;
  private final EngancheSubsystem engancheSubsystem;

  public EngancheCommand(EngancheSubsystem engancheSubsystem, DoubleSupplier enganchePower) {
    this.enganchePower = enganchePower;
    this.engancheSubsystem = engancheSubsystem;

    addRequirements(this.engancheSubsystem);
  }

  @Override
  public void execute() {
    engancheSubsystem.moveEnganche(Utils.applyDeadZone(enganchePower.getAsDouble(), DeadZone.ElevadorDeadZone));
  }

  @Override
  public void end(boolean isInterrupted) {
    engancheSubsystem.moveEnganche(0);
  }
}
