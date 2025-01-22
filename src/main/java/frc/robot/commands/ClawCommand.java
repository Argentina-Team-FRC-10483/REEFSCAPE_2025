package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.ClawSubsystem;

public class ClawCommand extends Command{
    
    private final ClawSubsystem CS;

    private final boolean action;

    public ClawCommand(ClawSubsystem Clawsubsystem , Boolean direction){
        action = direction;
        CS = Clawsubsystem;
        addRequirements(CS);
    }

    public int direction(boolean interuptor){
        if (interuptor) {
            int adelante = 1;
            return adelante;
        }
        else{
            int atras = 0;
            return atras;
        }
    }

    @Override
    public void initialize() {
    }

    @Override
    public void execute() {
        CS.translation(direction(action));
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
