package frc.robot.commands;

import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.PinzaSubsystem;

public class PinzaCommand extends Command{
    private final Joystick joystick = new Joystick(0);

    private final  PinzaSubsystem PS;

    boolean apertura = joystick.getRawButton(4);
    double succion = joystick.getRawAxis(2);
    double expulsar = joystick.getRawAxis(3);

    public PinzaCommand(PinzaSubsystem accion){
        PS = accion;
        addRequirements(PS);
    }

    public double validacionApertura(boolean producto){
        if (producto == true){
            return 1.0;
        }
        else{
            return 0.0;
        }
    }

    @Override
    public void initialize() {
        PS.aperturaPinza(validacionApertura(apertura));
        System.out.println("V");
    }

    @Override
    public void execute() {
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
