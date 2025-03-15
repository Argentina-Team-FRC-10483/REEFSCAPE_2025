package frc.robot.subsystems;

import com.revrobotics.spark.config.SparkMaxConfig;
import frc.robot.Constants.HandConstants;

public class InteriorRodSubsystem extends SingleMotorVelocitySubsystem {
  @Override
  protected int getCanId() {
    return HandConstants.INTERIOR_CAN_ID;
  }

  @Override
  protected SparkMaxConfig getMotorConfig() {
    return super.getMotorConfig();
  }
}
