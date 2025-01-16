package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.ArmSubsystem;

public class ArmCommand extends Command {
    
    private final ArmSubsystem AS;

    private final double height;
    
    public ArmCommand(ArmSubsystem armsubsystem, Double size){
        height = size;
        AS = armsubsystem; 
        addRequirements(AS);
    }

    @Override
    public void initialize() {
    }

    @Override
    public void execute() {
        AS.armMove(height);
    }

    @Override
    public boolean isFinished() {
        return false;
    }

    @Override
    public void end(boolean interrupted) {
        AS.armMove(0);
    }
}
