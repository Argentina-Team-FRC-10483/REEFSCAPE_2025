package frc.robot.commands;

import java.util.function.DoubleSupplier;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.DeadZone;
import frc.robot.subsystems.ElevadorSubsystem;

public class ElevadorCommand extends Command {
  private final DoubleSupplier potenciaElevador;
  private final ElevadorSubsystem elevadorSubsystem;

  public ElevadorCommand(ElevadorSubsystem elevadorSubsystem, DoubleSupplier potenciaElevador) {
    this.potenciaElevador = potenciaElevador;
    this.elevadorSubsystem = elevadorSubsystem;

    addRequirements(this.elevadorSubsystem);
  }

  @Override
  public void initialize() {
  }

  @Override
  public void execute() {
    double potencia = potenciaElevador.getAsDouble();
    if (potencia <= DeadZone.ElevadorDeadZone && potencia > 0) {
      potencia = 0;
    } else if(potencia >= -DeadZone.ElevadorDeadZone && potencia < 0) {
      potencia = 0;
    }
    elevadorSubsystem.moverElevador(potencia);
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
