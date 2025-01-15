package frc.robot.subsystems;

import edu.wpi.first.wpilibj.motorcontrol.Spark;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class PinzaSubsystem extends SubsystemBase{
    
    private final Spark articulacion = new Spark(0);
    private final Spark motorLeft = new Spark(0);
    private final Spark motorRight = new Spark(0);

    public void aperturaPinza(double estado){
        articulacion.set(estado);
    }

    public void traslado(double direccion){
        motorLeft.set(direccion);
        motorRight.set(direccion);
    }
}