package frc.robot.subsystems;
import edu.wpi.first.wpilibj.motorcontrol.Spark;
import frc.robot.Constants.ArmConstants;
import edu.wpi.first.wpilibj2.command.SubsystemBase;


public class ArmSubsystem extends SubsystemBase{

private final Spark motorShoulder = new Spark(ArmConstants.ArmMotorShoulder_ID);

    public void armMove(double angle){
        motorShoulder.set(angle);
    }
}
