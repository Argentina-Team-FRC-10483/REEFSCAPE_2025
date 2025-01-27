package frc.robot.subsystems;

import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkLowLevel.MotorType;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.PinzaConstants;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class ClawSubsystem extends SubsystemBase{            

    private final SparkMax motorCenter;
    private final SparkMax motorRight;
    
    private long debugTimeClawSub = 0;

    public ClawSubsystem(){
        this.motorCenter = new SparkMax(PinzaConstants.PinzaMotorRodilloCentral_ID, MotorType.kBrushless);
        this.motorRight = new SparkMax(PinzaConstants.PinzaMotorRodilloDerecho_ID, MotorType.kBrushless);
    }

    @Override
    public void periodic(){
        SmartDashboard.putNumber("Potencia del lado Central", motorCenter.get());
        SmartDashboard.putNumber("Potencia del lado Derecho", motorRight.get());
    }

    public void recoleccion (double direction){
        this.motorRight.set(direction);
    }

    public void soltar (double direction){
        this.motorCenter.set(direction);
    }
}