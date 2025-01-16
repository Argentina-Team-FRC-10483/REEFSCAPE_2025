package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.ClabSubsystem;

public class ClabCommand extends Command{
    
    private final ClabSubsystem CS;

    private final double action;

    public ClabCommand(ClabSubsystem clabsubsystem , double direction){
        action = direction;
        CS = clabsubsystem;
        addRequirements(CS);
    }

    @Override
    public void initialize() {
    }

    @Override
    public void execute() {
        CS.translation(action);
    }

    @Override
    public boolean isFinished() {
        return false;
    }

    @Override
    public void end(boolean interrupted) {
        CS.translation(0);
    }
}
