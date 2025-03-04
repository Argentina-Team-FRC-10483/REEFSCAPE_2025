package frc.robot.subsystems;

import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.SparkMaxConfig;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.HandConstants;
import frc.robot.Constants.NEOMotorsConstants;

public class RodLateralesSubsystem extends SubsystemBase {
  private final SparkMax motor;


  public RodLateralesSubsystem() {
    motor = new SparkMax(HandConstants.ROD_LATERALES_ID, MotorType.kBrushless);
    SparkMaxConfig motorConfig = new SparkMaxConfig();
    motorConfig.voltageCompensation(NEOMotorsConstants.VOLTAGE_COMPENSATION_NEO);
    motorConfig.smartCurrentLimit(NEOMotorsConstants.CURRENT_LIMIT_NEO);
    motor.configure(motorConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
  }

  @Override
  public void periodic() {

  }

  public void andarRodillo(double power) {
    motor.set(power);
  }

}
