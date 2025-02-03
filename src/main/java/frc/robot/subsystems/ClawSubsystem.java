package frc.robot.subsystems;

import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.SparkBaseConfig;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkMaxConfig;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.PinzaConstants;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class ClawSubsystem extends SubsystemBase{            

    private final SparkMax motorCenter;
    private final SparkMax motorRight;
    
    private long debugTimeClawSub = 0;

    public ClawSubsystem(){
        //Motores
        this.motorCenter = new SparkMax(PinzaConstants.PinzaMotorRodilloCentral_ID, MotorType.kBrushless);
        this.motorRight = new SparkMax(PinzaConstants.PinzaMotorRodilloDerecho_ID, MotorType.kBrushless);
        //Configuraciones
        SparkMaxConfig config = new SparkMaxConfig();
        config.idleMode(IdleMode.kBrake);
        config.smartCurrentLimit(30, 1, 11000 );
        config.voltageCompensation(12);
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