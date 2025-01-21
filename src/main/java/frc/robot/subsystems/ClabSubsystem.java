package frc.robot.subsystems;

import edu.wpi.first.wpilibj.motorcontrol.Spark;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.PinzaConstants;

public class ClabSubsystem extends SubsystemBase{            
    
    private final Spark joint = new Spark(PinzaConstants.PinzaMotorArticulacionCentral_ID);
    private final Spark motorLeft = new Spark(PinzaConstants.PinzaMotorRodilloIzquierdo_ID);

    public boolean ClabState = false;

    public void aperturaPinza(){
        if (!ClabState){
            joint.set(1);
        }
        else{
            joint.set(0);
        }
    }

    public void translation (double direction){
        motorLeft.set(direction);
    }
}