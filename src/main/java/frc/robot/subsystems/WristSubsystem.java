package frc.robot.subsystems;

import com.revrobotics.spark.SparkLowLevel.MotorType;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

import com.revrobotics.spark.SparkMax;
import frc.robot.Constants.WristConstants;

public class WristSubsystem extends SubsystemBase{
    
    private final SparkMax motorWrist;

    public WristSubsystem(){
        this.motorWrist = new SparkMax(WristConstants.WristMotor_ID, MotorType.kBrushless);
    }

    public void Articulación(double angle){
        this.motorWrist.set(angle);
    }
}
