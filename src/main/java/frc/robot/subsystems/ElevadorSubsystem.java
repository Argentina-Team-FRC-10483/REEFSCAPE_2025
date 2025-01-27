package frc.robot.subsystems;

import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.SparkMaxConfig;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.ElevadorConstants;

public class ElevadorSubsystem extends SubsystemBase {
  private final SparkMax MotorElevadorIzquierdoLider;
  private final SparkMax MotorElevadorDerechoSeguidor;

  public ElevadorSubsystem() {
    MotorElevadorIzquierdoLider = new SparkMax(ElevadorConstants.MotorElevadorIzquierdoLider_ID, MotorType.kBrushless);
    MotorElevadorDerechoSeguidor = new SparkMax(ElevadorConstants.MotorElevadorDerechoSeguidor_ID, MotorType.kBrushless);

    // Motor líder
    SparkMaxConfig configLider = new SparkMaxConfig();
    configLider.voltageCompensation(ElevadorConstants.VOLTAGE_COMPENSATION);
    configLider.smartCurrentLimit(ElevadorConstants.CURRENT_LIMIT);
    MotorElevadorIzquierdoLider.configure(configLider, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);

    // Motor seguidor
    SparkMaxConfig configSeguidor = new SparkMaxConfig();
    configSeguidor.follow(MotorElevadorIzquierdoLider);
    MotorElevadorDerechoSeguidor.configure(configSeguidor, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
  }

  @Override
  public void periodic() {
  }

  public void moverElevador(double velocidad) {
    MotorElevadorIzquierdoLider.set(velocidad);
  }
}