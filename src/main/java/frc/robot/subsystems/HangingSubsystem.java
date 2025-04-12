/*
 * HangingSubsystem.java
 * 
 * Subsistema que controla el mecanismo de enganche (trepado) del robot.
 * Utiliza un motor NEO con SparkMax y control de límites suaves.
 */
package frc.robot.subsystems;

import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.SparkMaxConfig;
import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.DriveConstants;
import frc.robot.Constants.HangingConstants;
import frc.robot.Constants.NEOMotorsConstants;

/////////////////////////////////////////////////////////////
// SUBSISTEMA DE ENGANCHE (HANGING)
/////////////////////////////////////////////////////////////
public class HangingSubsystem extends SubsystemBase {

    /////////////////////////////////////////////////////////////
    // COMPONENTES PRINCIPALES
    /////////////////////////////////////////////////////////////
    private final SparkMax motor;                // Controlador SparkMax para el motor
    private final RelativeEncoder hangingEncoder; // Encoder del mecanismo
    private final SlewRateLimiter filter;        // Filtro para suavizar movimientos

    /////////////////////////////////////////////////////////////
    // CONSTANTES Y CONFIGURACIONES
    /////////////////////////////////////////////////////////////
    private static final double SLOWDOWN_RANGE = 1.0;  // Rango para reducción de velocidad (metros)
    private static final double UPPER_LIMIT = 47.0;    // Límite superior del mecanismo
    private static final double LOWER_LIMIT = -52.0;   // Límite inferior del mecanismo
    
    // Nombres para Dashboard
    public static final String DASH_POS = "Hanging/Pos";
    public static final String DASH_RESET_ENCODER = "Hanging/Reset Encoder";

    /////////////////////////////////////////////////////////////
    // CONSTRUCTOR
    /////////////////////////////////////////////////////////////
    public HangingSubsystem() {
        // Configuración del motor
        motor = new SparkMax(HangingConstants.CAN_ID, MotorType.kBrushless);
        motor.setCANTimeout(DriveConstants.CAN_TIMEOUT);
        motor.configure(getLeaderConfig(), ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);

        // Configuración del encoder
        hangingEncoder = motor.getEncoder();
        hangingEncoder.setPosition(0);

        // Configuración del filtro de movimiento
        filter = new SlewRateLimiter(1.5); // Limita la aceleración a 1.5 unidades/seg²

        // Botón para resetear encoder en Dashboard
        SmartDashboard.putData(DASH_RESET_ENCODER, new InstantCommand(() -> hangingEncoder.setPosition(0)));
    }

    /////////////////////////////////////////////////////////////
    // CONFIGURACIÓN DEL MOTOR
    /////////////////////////////////////////////////////////////
    private static SparkMaxConfig getLeaderConfig() {
        SparkMaxConfig leaderConfig = new SparkMaxConfig();

        // Configuración de límites suaves
        leaderConfig.softLimit
            .forwardSoftLimitEnabled(true)
            .forwardSoftLimit(UPPER_LIMIT)
            .reverseSoftLimitEnabled(true)
            .reverseSoftLimit(LOWER_LIMIT);

        // Configuración eléctrica
        leaderConfig
            .voltageCompensation(NEOMotorsConstants.VOLTAGE_COMPENSATION)
            .smartCurrentLimit(NEOMotorsConstants.CURRENT_LIMIT);

        return leaderConfig;
    }

    /////////////////////////////////////////////////////////////
    // MÉTODOS PERIÓDICOS
    /////////////////////////////////////////////////////////////
    @Override
    public void periodic() {
        // Actualización de valores en Dashboard
        SmartDashboard.putNumber(DASH_POS, getHangingPosition());
    }

    /////////////////////////////////////////////////////////////
    // MÉTODOS DE OBTENCIÓN DE ESTADO
    /////////////////////////////////////////////////////////////
    public double getHangingPosition() {
        return hangingEncoder.getPosition();
    }

    /////////////////////////////////////////////////////////////
    // MÉTODOS DE CONTROL
    /////////////////////////////////////////////////////////////
    /**
     * Mueve el mecanismo de enganche con control de velocidad
     * @param speed Velocidad deseada (-1.0 a 1.0)
     */
    public void move(double speed) {
        double currentPosition = getHangingPosition();

        // Control de reducción de velocidad cerca de los límites
        boolean inLowerSlowdownZone = currentPosition <= LOWER_LIMIT + SLOWDOWN_RANGE;
        boolean inUpperSlowdownZone = currentPosition >= UPPER_LIMIT - SLOWDOWN_RANGE;
        
        if (speed < 0 && inLowerSlowdownZone) 
            speed = speed * getLowerSlowdownFactor(currentPosition);
        if (speed > 0 && inUpperSlowdownZone) 
            speed = speed * getUpperSlowdownFactor(currentPosition);
        
        // Aplicar filtro y mover
        speed = filter.calculate(speed);
        motor.set(speed);
    }

    /////////////////////////////////////////////////////////////
    // MÉTODOS DE CÁLCULO DE VELOCIDAD
    /////////////////////////////////////////////////////////////
    private double getUpperSlowdownFactor(double currentPosition) {
        return Math.max((UPPER_LIMIT - currentPosition) / SLOWDOWN_RANGE, 0);
    }

    private double getLowerSlowdownFactor(double currentPosition) {
        return Math.max((currentPosition - LOWER_LIMIT) / SLOWDOWN_RANGE, 0);
    }
}