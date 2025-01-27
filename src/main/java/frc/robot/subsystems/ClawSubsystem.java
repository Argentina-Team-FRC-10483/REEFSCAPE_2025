package frc.robot.subsystems;

import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.PinzaConstants;

public class ClawSubsystem extends SubsystemBase{            

    private final SparkMax motorLeft;
    private final SparkMax motorRight;
    
    public int clawState;

    public ClawSubsystem(){
        this.motorLeft = new SparkMax(PinzaConstants.PinzaMotorRodilloIzquierdo_ID, MotorType.kBrushless);
        this.motorRight = new SparkMax(PinzaConstants.PinzaMotorRodilloDerecho_ID, MotorType.kBrushless);
    }

    public void translation (int direction){
        this.motorLeft.set(direction);
        this.motorRight.set(direction);
    }
}