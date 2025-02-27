package frc.robot.subsystems;

import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.SparkMaxConfig;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.AlgaeIntakeConstants;

public class RodLateralesSubsystem extends SubsystemBase {
  private final SparkMax rodillosLateralesMotor;


  public RodLateralesSubsystem() {
    rodillosLateralesMotor = new SparkMax(AlgaeIntakeConstants.RodilloMotor_ID, MotorType.kBrushless);
    SparkMaxConfig rodillosLateralesMotorConfig = new SparkMaxConfig();
    rodillosLateralesMotorConfig.voltageCompensation(AlgaeIntakeConstants.RodilloMotor_CompVolt);
    rodillosLateralesMotorConfig.smartCurrentLimit(AlgaeIntakeConstants.RodilloMotor_LIMITE);
    rodillosLateralesMotor.configure(rodillosLateralesMotorConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
  }

  @Override
  public void periodic() {

  }

  public void andarRodillo(double power) {
    rodillosLateralesMotor.set(power);
  }

}
