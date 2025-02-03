package frc.robot.subsystems;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.SparkMaxConfig;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.AbsoluteEncoder;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

import frc.robot.Constants.ArmConstants;
import edu.wpi.first.wpilibj2.command.SubsystemBase;


public class ArmSubsystem extends SubsystemBase{

    private final SparkMax motorShoulder;
    private final AbsoluteEncoder encoder;
    private long debugTimeArmSub = 0;

    public ArmSubsystem(){
        //Motor
        this.motorShoulder = new SparkMax(ArmConstants.ArmMotorShoulder_ID, MotorType.kBrushless);
        // Encoder
        this.encoder = motorShoulder.getAbsoluteEncoder(); 
        double position = encoder.getPosition();
        double pulsos = position * 42;
        double velocidad = encoder.getVelocity();
        //Configuraciones
        SparkMaxConfig config = new SparkMaxConfig();
        config.idleMode(IdleMode.kBrake);
        config.smartCurrentLimit(40, 1,5676);
        config.voltageCompensation(12);
    }

    @Override
    public void periodic(){
        SmartDashboard.putNumber("Motor Brazo", motorShoulder.get());
    }

    public void armMove(double torque){
        motorShoulder.set(torque);
    }
}