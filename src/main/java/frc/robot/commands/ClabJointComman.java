package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.ClabSubsystem;

public class ClabJointComman extends Command{

    private final  ClabSubsystem CS;
    
    private final boolean size;

    public ClabJointComman(ClabSubsystem clabsubsystem, Boolean apertura){
        size = apertura;
        CS = clabsubsystem; 
        addRequirements(CS);
    }

    @Override
    public void initialize() {
        CS.ClabState = size;
    }

    @Override
    public void execute() {
        CS.ClabState = size;
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
