package frc.robot.subsystems;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import frc.robot.Constants.ArmConstants;
import edu.wpi.first.wpilibj2.command.SubsystemBase;


public class ArmSubsystem extends SubsystemBase{

    private final SparkMax motorShoulder;

public ArmSubsystem(){
    this.motorShoulder = new SparkMax(ArmConstants.ArmMotorShoulder_ID, MotorType.kBrushless);
}

    public void armMove(double angle){
        motorShoulder.set(angle);
    }
}
