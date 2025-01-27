package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.ElevadorSubsystem;

public class ElevadorCommand extends Command {
  private final double potenciaElevador;
  private final ElevadorSubsystem elevadorSubsystem;

  public ElevadorCommand(ElevadorSubsystem elevadorSubsystem, double potenciaElevador) {
    this.potenciaElevador = potenciaElevador;
    this.elevadorSubsystem = elevadorSubsystem;

    addRequirements(this.elevadorSubsystem);
  }

  @Override
  public void initialize() {
  }

  @Override
  public void execute() {
    elevadorSubsystem.moverElevador(potenciaElevador);
  }

  @Override
  public void end(boolean isInterrupted) {
    elevadorSubsystem.moverElevador(0);
  }

  @Override
  public boolean isFinished() {
    return false;
  }
}
