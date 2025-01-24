package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.WristSubsystem;

public class WristCommand extends Command{
    
    private final WristSubsystem WS; 
    private final double articulacion;

    public WristCommand(WristSubsystem Wristsubsystem, double angle){
        articulacion = angle;
        WS = Wristsubsystem;
        addRequirements(WS);
    }

    @Override
    public void initialize() {
    }

    @Override
    public void execute() {
        WS.Articulación(articulacion);
    }

    @Override
    public boolean isFinished() {
        return false;
    }

    @Override
    public void end(boolean interrupted) {
        WS.Articulación(0);
    }
}
