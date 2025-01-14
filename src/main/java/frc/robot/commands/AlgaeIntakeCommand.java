package frc.robot.commands;

import java.util.function.BooleanSupplier;
import java.util.function.DoubleSupplier;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.AlgaeIntakeSubsystem;

public class AlgaeIntakeCommand extends Command{
    private final double power;
    private final BooleanSupplier button;

    private final AlgaeIntakeSubsystem algaeIntakeSubsystem;

    public AlgaeIntakeCommand(BooleanSupplier button, AlgaeIntakeSubsystem algaeIntakeSubsystem, double power){
        this.power = power;
        this.button = button;
        this.algaeIntakeSubsystem = algaeIntakeSubsystem;

        addRequirements(this.algaeIntakeSubsystem);
    }

    @Override
    public void initialize() {
  }

  @Override
    public void execute() {
    algaeIntakeSubsystem.andarRodillo(power);
  }

  @Override
    public void end(boolean isInterrupted) {
    algaeIntakeSubsystem.andarRodillo(0);
  }

  @Override
  public boolean isFinished() {
    return !this.button.getAsBoolean();
  }
}
