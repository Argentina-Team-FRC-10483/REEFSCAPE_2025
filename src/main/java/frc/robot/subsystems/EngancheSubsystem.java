package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.EngancheContants;

import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.SparkMaxConfig;

public class EngancheSubsystem extends SubsystemBase {
    private SparkMax motorEnganche;

    public void Enganche() {
        motorEnganche = new SparkMax(EngancheContants.MotorEnganche_ID,MotorType.kBrushless);
        SparkMaxConfig motorEngancheConfig = new SparkMaxConfig();
        motorEngancheConfig.voltageCompensation(EngancheContants.MotorEnganche_CompVolt);
        motorEngancheConfig.smartCurrentLimit(EngancheContants.MotorEnganche_LIMITE);
        motorEnganche.configure(motorEngancheConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
    }
    public void enganchar(double vel) {
        motorEnganche.set(vel);
    }

    @Override
    public void periodic() { 
     
    }

}

