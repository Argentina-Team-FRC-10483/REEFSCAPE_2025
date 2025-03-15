package frc.robot.subsystems;

import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.SparkMaxConfig;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.HandConstants;
import frc.robot.Constants.NEOMotorsConstants;

public class RodInteriorSubsystem extends SubsystemBase {
  private final SparkMax motor;
  private final RelativeEncoder rodInteriorEncoder;

  public RodInteriorSubsystem() {
    motor = new SparkMax(HandConstants.INTERIOR_CAN_ID, MotorType.kBrushless);
    SparkMaxConfig motorConfig = new SparkMaxConfig();
    motorConfig.voltageCompensation(NEOMotorsConstants.VOLTAGE_COMPENSATION);
    motorConfig.smartCurrentLimit(NEOMotorsConstants.CURRENT_LIMIT);
    motor.configure(motorConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
    rodInteriorEncoder = motor.getEncoder();
  }

  public double getRodInteriorPosition() {
    return rodInteriorEncoder.getPosition();
  }

  @Override
  public void periodic() {
    SmartDashboard.putNumber("Rodillo interior posicion", getRodInteriorPosition());
  }

  public void andarRodillo(double power) {
    motor.set(power);
  }

}
