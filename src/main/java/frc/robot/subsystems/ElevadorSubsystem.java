package frc.robot.subsystems;

import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.SparkMaxConfig;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.ElevadorConstants;

public class ElevadorSubsystem extends SubsystemBase {
  private final SparkMax MotorElevadorIzquierdoLider;
  private final SparkMax MotorElevadorDerechoSeguidor;
  private final RelativeEncoder encoderElevador;

  private static final double LIMITE_SUPERIOR = 100.0;
  private static final double LIMITE_INFERIOR = 0.0;

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

        // Obtener el encoder
        encoderElevador = MotorElevadorIzquierdoLider.getEncoder();

        // Resetear el encoder al iniciar
        encoderElevador.setPosition(0);

        SmartDashboard.putData("Reiniciar Encoder Elevador", new InstantCommand(() -> encoderElevador.setPosition(0)));
  }

  @Override
  public void periodic() {
    SmartDashboard.putNumber("Elevador Posicion", getPosicionElevador());
  }

  public double getPosicionElevador() {
    return encoderElevador.getPosition();
  }

  public void moverElevador(double velocidad) {
    double posicionActual = getPosicionElevador();

    if ((velocidad > 0 && posicionActual >= LIMITE_SUPERIOR) || 
        (velocidad < 0 && posicionActual <= LIMITE_INFERIOR)) {
          
      MotorElevadorIzquierdoLider.set(0);
    } else {
      MotorElevadorIzquierdoLider.set(velocidad);
    }
  }
}