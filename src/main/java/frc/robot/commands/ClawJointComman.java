package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.ClawSubsystem;

public class ClawJointComman extends Command{

    private final  ClawSubsystem CS;
    
    private final int size;

    public ClawJointComman(ClawSubsystem Clawsubsystem, int apertura){
        size = apertura;
        CS = Clawsubsystem; 
        addRequirements(CS);
    }

    @Override
    public void initialize() {
        CS.clawState = size;
    }

    @Override
    public void execute() {
        CS.clawState = size;
    }

    @Override
    public boolean isFinished() {
        return false;
    }

    @Override
    public void end(boolean interrupted) {
        CS.clawState = 0;
    }
}
