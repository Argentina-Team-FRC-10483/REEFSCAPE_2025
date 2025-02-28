package frc.robot.subsystems;

import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.SparkMaxConfig;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.AlgaeIntakeConstants;

public class RodInteriorSubsystem extends SubsystemBase {
  private final SparkMax rodilloIntMotor;


  public RodInteriorSubsystem() {
    rodilloIntMotor = new SparkMax(AlgaeIntakeConstants.RodilloMotor_ID, MotorType.kBrushless);
    SparkMaxConfig rodilloIntMotorConfig = new SparkMaxConfig();
    rodilloIntMotorConfig.voltageCompensation(AlgaeIntakeConstants.RodilloMotor_CompVolt);
    rodilloIntMotorConfig.smartCurrentLimit(AlgaeIntakeConstants.RodilloMotor_LIMITE);
    rodilloIntMotor.configure(rodilloIntMotorConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
  }

  @Override
  public void periodic() {

  }

  public void andarRodilloInterior(double powerRodInterior) {
    rodilloIntMotor.set(powerRodInterior);
  }

}
