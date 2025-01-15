package frc.robot.subsystems;

import edu.wpi.first.wpilibj.motorcontrol.Spark;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.PinzaConstants;

public class PinzaSubsystem extends SubsystemBase{
    
    private final Spark articulacion = new Spark(PinzaConstants.PinzaMotorArticulacionCentral_ID);
    private final Spark motorLeft = new Spark(PinzaConstants.PinzaMotorRodilloIzquierdo_ID);
    private final Spark motorRight = new Spark(PinzaConstants.PinzaMotorRodilloDerecho_ID);

    public boolean EstadoPinza = false;

    public void aperturaPinza(double estado){
        if (!EstadoPinza){
            articulacion.set(-1);
        }
        else if (EstadoPinza){
            articulacion.set(1);
        }
        else{
            articulacion.set(0);
        }
    }

    public void traslado(double direccion){
        motorLeft.set(direccion);
        motorRight.set(direccion);
    }
}