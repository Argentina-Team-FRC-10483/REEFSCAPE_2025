package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.AlgaeIntakeSubsystem;

public class AlgaInTakeArmCommand extends Command{
    private final double powerArm;
    private final AlgaeIntakeSubsystem algaeIntakeSubsystem;

    public AlgaInTakeArmCommand(AlgaeIntakeSubsystem algaeIntakeSubsystem, double powerArm){
        this.powerArm = powerArm;
        this.algaeIntakeSubsystem = algaeIntakeSubsystem;

        addRequirements(this.algaeIntakeSubsystem);
    }
   @Override
   public void initialize() {
}

@Override
  public void execute() {
  algaeIntakeSubsystem.brazoRodillo(powerArm);
}

@Override
  public void end(boolean isInterrupted) {
  algaeIntakeSubsystem.brazoRodillo(0);
}

@Override
public boolean isFinished() {
  return false;
}
}
