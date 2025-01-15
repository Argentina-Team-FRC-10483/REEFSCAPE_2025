package frc.robot.commands;

import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.PinzaSubsystem;
import frc.robot.Constants.JoystickMappings;

public class PinzaCommand extends Command{
    private final Joystick joystick = new Joystick(0);  

    private final  PinzaSubsystem PS;

    boolean apertura = joystick.getRawButton(JoystickMappings.PinzaButtonApertura_ID);
    double succion = joystick.getRawAxis(JoystickMappings.PinzaAxisSuccion_ID);
    double expulsar = joystick.getRawAxis(JoystickMappings.PinzaAxisExpulsar_ID); 

    public PinzaCommand(PinzaSubsystem accion){
        PS = accion;
        addRequirements(PS);
    }

    @Override
    public void initialize() {
        PS.EstadoPinza = apertura;
    }

    @Override
    public void execute() {
        PS.EstadoPinza = apertura;
        PS.traslado(succion);
    }

    @Override
    public boolean isFinished() {
        return false;
    }

    @Override
    public void end(boolean interrupted) {
        PS.traslado(0);
    }
}
