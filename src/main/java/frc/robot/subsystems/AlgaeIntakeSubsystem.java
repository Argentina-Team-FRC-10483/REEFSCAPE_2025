package frc.robot.subsystems;

import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.SparkMaxConfig;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.AlgaeIntakeConstants;
import frc.robot.Constants.DebugConstants;

/**
 * Subsystem que controla el rodillo de la toma de algas (AlgaeIntake).
 * Este subsistema usa un motor SparkMax configurado en modo brushed.
 */
public class AlgaeIntakeSubsystem extends SubsystemBase {
    // Motor controlador para el rodillo
    private final SparkMax rodilloMotor;

    private long debugTimeAlgaeIntakeSub = 0;

    /**
     * Constructor del subsistema. Configura el motor con los parámetros especificados en las constantes.
     */
    public AlgaeIntakeSubsystem(){
        rodilloMotor = new SparkMax(AlgaeIntakeConstants.RodilloMotor_ID, MotorType.kBrushed); // Inicializa el SparkMax con su ID y tipo de motor (brushed)

        // Configuración del motor usando un objeto SparkMaxConfig
        SparkMaxConfig rodilloMotorConfig = new SparkMaxConfig();
        rodilloMotorConfig.voltageCompensation(AlgaeIntakeConstants.RodilloMotor_CompVolt); // Compensación del voltaje
        rodilloMotorConfig.smartCurrentLimit(AlgaeIntakeConstants.RodilloMotor_LIMITE); // Límite de corriente

        // Aplica la configuración al motor y asegura que los parámetros sean seguros
        rodilloMotor.configure(rodilloMotorConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);

        // Mensaje de debugging para confirmar la inicialización
        System.out.println("AlgaeIntakeSubsystem inicializado con ID: " + AlgaeIntakeConstants.RodilloMotor_ID);
    }
    /**
     * Método que se llama periódicamente
     */
    @Override
    public void periodic(){
        long currentTime = System.currentTimeMillis();
        if (currentTime - debugTimeAlgaeIntakeSub >= DebugConstants.DEBUG_INTERVAL_MS) {
            debugTimeAlgaeIntakeSub = currentTime; // Actualiza el tiempo del último mensaje
            System.out.println("AlgaeIntakeSubsystem está operativo. Potencia actual del rodillo: " + rodilloMotor.get());
        }

        SmartDashboard.putNumber("Potencia rodillo", rodilloMotor.get());
    }

    public void andarRodillo(double power){
        rodilloMotor.set(power);
    }
}
