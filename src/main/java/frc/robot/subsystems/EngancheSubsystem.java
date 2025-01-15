import edu.wpi.first.wpilibj2.command.SubsystemBase;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkLowLevel.MotorType;

public class EngancheSubsystem extends SubsystemBase {
    private final SparkMax MotorEnganche;

    public void Enganche {
        MotorEnganche = new SparkMax(EngancheContants.MotorEnganche_ID,MotorType.);
    }
    public void Enganche(double vel) {
        MotorEnganche.set(vel);
    }

    @Override
    public void periodic() {
     
    }

}

