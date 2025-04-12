/*
 * ArmSubsystem.java
 * 
 * Subsistema que controla el brazo/articulación del robot con:
 * - Control de posición mediante SparkMax
 * - Límites de movimiento seguros
 * - Configuración PID para movimiento preciso
 */
package frc.robot.subsystems;

import com.revrobotics.spark.SparkBase;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.SparkBaseConfig;
import com.revrobotics.spark.config.SparkMaxConfig;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.DriveConstants;
import frc.robot.Constants.ArmConstants;
import frc.robot.Constants.NEOMotorsConstants;

/////////////////////////////////////////////////////////////
// SUBSISTEMA DE BRAZO/ARTICULACIÓN
/////////////////////////////////////////////////////////////
public class ArmSubsystem extends SubsystemBase implements MovableSubsystem {

    /////////////////////////////////////////////////////////////
    // COMPONENTES PRINCIPALES
    /////////////////////////////////////////////////////////////
    private final SparkMax motor;                     // Controlador SparkMax
    private final SparkClosedLoopController controller; // Controlador de lazo cerrado
    private final RelativeEncoder armEncoder;        // Encoder del brazo

    /////////////////////////////////////////////////////////////
    // VARIABLES DE ESTADO
    /////////////////////////////////////////////////////////////
    private double position;                         // Posición objetivo actual

    /////////////////////////////////////////////////////////////
    // CONSTANTES Y LÍMITES
    /////////////////////////////////////////////////////////////
    public static double UPPER_LIMIT = -1.6;         // Límite superior (posición más alta)
    public static double LOWER_LIMIT = -22;          // Límite inferior (posición más baja)
    
    // Nombres para Dashboard
    public static final String DASH_ARM_POS = "Arm/Pos";
    public static final String DASH_ARM_TARGET = "Arm/Target";
    public static final String DASH_RESET_ARM_ENCODER = "Arm/Reset Encoder";

    /////////////////////////////////////////////////////////////
    // CONSTRUCTOR
    /////////////////////////////////////////////////////////////
    public ArmSubsystem() {
        // Configuración del motor
        motor = new SparkMax(ArmConstants.CAN_ID, MotorType.kBrushless);
        motor.setCANTimeout(DriveConstants.CAN_TIMEOUT);
        
        // Configuración del controlador
        controller = motor.getClosedLoopController();
        motor.configure(getLeaderConfig(), ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
        
        // Configuración del encoder
        armEncoder = motor.getEncoder();
        reset();
        
        // Botón para resetear encoder en Dashboard
        SmartDashboard.putData(DASH_RESET_ARM_ENCODER, new InstantCommand(this::reset));
    }

    /////////////////////////////////////////////////////////////
    // CONFIGURACIÓN DEL MOTOR
    /////////////////////////////////////////////////////////////
    private static SparkMaxConfig getLeaderConfig() {
        SparkMaxConfig leaderConfig = new SparkMaxConfig();

        // Configuración de control PID
        leaderConfig.closedLoop
            .p(0.02)        // Ganancia proporcional
            .i(0.00004)     // Ganancia integral (muy pequeña para ajuste fino)
            .iZone(1.7)     // Zona integral (rango donde actúa el término I)
            .d(0)           // Ganancia derivativa
            .maxMotion
                .maxVelocity(0.4)       // Velocidad máxima (conservadora)
                .allowedClosedLoopError(1) // Error permitido
                .maxAcceleration(0.05); // Aceleración máxima

        // Configuración general del motor
        leaderConfig
            .voltageCompensation(NEOMotorsConstants.VOLTAGE_COMPENSATION) // Compensación de voltaje
            .smartCurrentLimit(NEOMotorsConstants.CURRENT_LIMIT)          // Límite de corriente
            .idleMode(SparkBaseConfig.IdleMode.kBrake);                   // Modo freno

        return leaderConfig;
    }

    /////////////////////////////////////////////////////////////
    // MÉTODOS DE LA INTERFAZ MOVABLESUBSYSTEM
    /////////////////////////////////////////////////////////////
    
    @Override
    public void reset() {
        armEncoder.setPosition(0);
        position = 0;
        controller.setReference(this.position, SparkBase.ControlType.kPosition);
    }

    @Override
    public double getActualPosition() {
        return armEncoder.getPosition();
    }

    @Override
    public double getTargetPosition() {
        return position;
    }

    @Override
    public void move(double position) {
        this.position = MathUtil.clamp(position, LOWER_LIMIT, UPPER_LIMIT);
    }

    /////////////////////////////////////////////////////////////
    // MÉTODOS PERIÓDICOS
    /////////////////////////////////////////////////////////////
    @Override
    public void periodic() {
        // Actualización de valores en Dashboard
        SmartDashboard.putNumber(DASH_ARM_POS, getActualPosition());
        SmartDashboard.putNumber(DASH_ARM_TARGET, position);
        
        // Actualizar referencia de posición del controlador
        controller.setReference(this.position, SparkBase.ControlType.kPosition);
    }
}