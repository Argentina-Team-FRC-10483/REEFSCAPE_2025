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

    @Override
    public void initialize() {
    }

    @Override
    public void execute() {
        CS.recoleccion(action ? 0.75 : 0);
        CS.soltar(action ? 0 : -0.75);

        // bool ? : 
    }

    @Override
    public boolean isFinished() {
        return false;
    }

    @Override
    public void end(boolean interrupted) {
        CS.recoleccion(0);
        CS.soltar(0);
    }
}
