package frc.robot.subsystems;

import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.SparkMaxConfig;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.AlgaeIntakeConstants;
import frc.robot.Constants.NEOMotorsConstants;

public class AlgaeIntakeSubsystem extends SubsystemBase {
  private final SparkMax rodilloMotor;


  public AlgaeIntakeSubsystem() {
    rodilloMotor = new SparkMax(AlgaeIntakeConstants.RodilloMotor_ID, MotorType.kBrushless);
    SparkMaxConfig rodilloMotorConfig = new SparkMaxConfig();
    rodilloMotorConfig.voltageCompensation(NEOMotorsConstants.VOLTAGE_COMPENSATION_NEO);
    rodilloMotorConfig.smartCurrentLimit(NEOMotorsConstants.CURRENT_LIMIT_NEO);
    rodilloMotor.configure(rodilloMotorConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
  }

  public void andarRodillo(double power) {
    rodilloMotor.set(power);
  }
}
