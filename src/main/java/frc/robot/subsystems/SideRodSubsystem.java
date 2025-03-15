package frc.robot.subsystems;

import com.revrobotics.spark.config.SparkMaxConfig;
import frc.robot.Constants;

public class SideRodSubsystem extends SingleMotorVelocitySubsystem {
  @Override
  protected int getCanId() {
    return Constants.HandConstants.SIDE_CAN_ID;
  }

  @Override
  protected SparkMaxConfig getMotorConfig() {
    return super.getMotorConfig();
  }
}
