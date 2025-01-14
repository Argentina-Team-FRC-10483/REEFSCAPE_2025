package frc.robot.commands;

import java.util.function.DoubleSupplier;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.AlgaeIntakeSubsystem;

public class AlgaeIntakeCommand extends Command{
    private final DoubleSupplier adelante;
    private final DoubleSupplier atras;

    private final AlgaeIntakeSubsystem algaeIntakeSubsystem;

    public AlgaeIntakeCommand(DoubleSupplier adelante, DoubleSupplier atras, AlgaeIntakeSubsystem algaeIntakeSubsystem){
        this.adelante = adelante;
        this.atras = atras;
        this.algaeIntakeSubsystem = algaeIntakeSubsystem;

        addRequirements(this.algaeIntakeSubsystem);
    }

    @Override
    public void initialize() {
  }

  @Override
    public void execute() {
    algaeIntakeSubsystem.andarRodillo(adelante.getAsDouble(), atras.getAsDouble());
  }

  @Override
    public void end(boolean isInterrupted) {
  }

  @Override
  public boolean isFinished() {
    return false;
  }
}
