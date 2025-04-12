/*
 * RodLateralesSubsystem.java
 * 
 * Subsistema que controla el/los rodillo(s) laterales del robot
 * Utiliza motor(es) NEO controlados por SparkMax
 */
package frc.robot.subsystems;

import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.SparkMaxConfig;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.HandConstants;
import frc.robot.Constants.NEOMotorsConstants;

/////////////////////////////////////////////////////////////
// SUBSISTEMA RODILLOS LATERALES
/////////////////////////////////////////////////////////////
public class RodLateralesSubsystem extends SubsystemBase {

    /////////////////////////////////////////////////////////////
    // COMPONENTES DEL SUBSISTEMA
    /////////////////////////////////////////////////////////////
    private final SparkMax motor;  // Controlador SparkMax para el motor NEO

    /////////////////////////////////////////////////////////////
    // CONSTRUCTOR
    /////////////////////////////////////////////////////////////
    /**
     * Configura el subsistema de rodillos laterales:
     * - Inicializa el controlador SparkMax
     * - Aplica configuración de seguridad
     * - Configura compensación de voltaje y límite de corriente
     */
    public RodLateralesSubsystem() {
        // Inicialización del motor NEO con SparkMax
        motor = new SparkMax(HandConstants.SIDE_CAN_ID, MotorType.kBrushless);
        
        // Configuración del motor
        SparkMaxConfig motorConfig = new SparkMaxConfig();
        motorConfig.voltageCompensation(NEOMotorsConstants.VOLTAGE_COMPENSATION); // 12V nominal
        motorConfig.smartCurrentLimit(NEOMotorsConstants.CURRENT_LIMIT);          // Límite de corriente
        
        // Aplicar configuración con reset seguro
        motor.configure(motorConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
    }

    /////////////////////////////////////////////////////////////
    // MÉTODOS DE CONTROL
    /////////////////////////////////////////////////////////////
    
    /**
     * Controla la velocidad del rodillo lateral
     * @param power Valor de potencia (-1.0 a 1.0)
     *             Positivo: rodillo hacia adelante
     *             Negativo: rodillo hacia atrás
     */
    public void andarRodillo(double power) {
        motor.set(power);
    }
}