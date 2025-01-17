package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.EngancheContants;

import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkLowLevel.MotorType;

public class EngancheSubsystem extends SubsystemBase {
    private SparkMax motorEnganche;

    public void Enganche() {
        motorEnganche = new SparkMax(EngancheContants.MotorEnganche_ID,MotorType.kBrushless);
    }
    public void enganchar(double vel) {
        motorEnganche.set(vel);
    }

    @Override
    public void periodic() { 
     
    }

}

