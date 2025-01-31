package frc.robot.subsystems;

import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.SparkMaxConfig;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.AlgaeIntakeConstants;

public class AlgaeIntakeSubsystem extends SubsystemBase {
    private final SparkMax rodilloMotor;
    

    public AlgaeIntakeSubsystem(){
        rodilloMotor = new SparkMax(AlgaeIntakeConstants.RodilloMotor_ID, MotorType.kBrushless);
        SparkMaxConfig rodilloMotorConfig = new SparkMaxConfig();
        rodilloMotorConfig.voltageCompensation(AlgaeIntakeConstants.RodilloMotor_CompVolt);
        rodilloMotorConfig.smartCurrentLimit(AlgaeIntakeConstants.RodilloMotor_LIMITE);
        rodilloMotor.configure(rodilloMotorConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);

    }

    @Override
    public void periodic(){

    }

    public void andarRodillo(double power){
        rodilloMotor.set(power);
    }

}
