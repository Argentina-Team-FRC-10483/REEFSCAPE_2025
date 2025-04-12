/*
 * ElevatorSubsystem.java
 * 
 * Subsistema que controla el elevador del robot con:
 * - Control de posición con SparkMax
 * - Límites de movimiento seguros
 * - Integración con el subsistema de brazo/muñeca
 * - Restricciones de seguridad cuando el brazo está en posiciones bajas
 */
package frc.robot.subsystems;

import com.revrobotics.spark.SparkBase;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.ClosedLoopConfig;
import com.revrobotics.spark.config.SparkBaseConfig;
import com.revrobotics.spark.config.SparkMaxConfig;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj.RobotState;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.DriveConstants;
import frc.robot.Constants.ElevatorConstants;
import frc.robot.Constants.NEOMotorsConstants;

/////////////////////////////////////////////////////////////
// SUBSISTEMA DE ELEVADOR
/////////////////////////////////////////////////////////////
public class ElevatorSubsystem extends SubsystemBase implements MovableSubsystem {

    /////////////////////////////////////////////////////////////
    // COMPONENTES PRINCIPALES
    /////////////////////////////////////////////////////////////
    private final SparkMax leftMotorLeader;       // Motor líder izquierdo
    private final SparkMax rightMotorFollow;      // Motor seguidor derecho
    private final SparkClosedLoopController controller; // Controlador de lazo cerrado
    private final RelativeEncoder elevatorEncoder; // Encoder del elevador
    private final ArmSubsystem armSubsystem;      // Referencia al subsistema de brazo
    
    /////////////////////////////////////////////////////////////
    // VARIABLES DE ESTADO
    /////////////////////////////////////////////////////////////
    private double position = 0;                 // Posición objetivo actual

    /////////////////////////////////////////////////////////////
    // CONSTANTES Y LÍMITES
    /////////////////////////////////////////////////////////////
    public static double UPPER_LIMIT = 88.0;               // Límite superior absoluto
    public static double LOWER_LIMIT = 0.0;                // Límite inferior absoluto
    public static double ARM_THRESHOLD = -18;              // Ángulo de brazo que activa restricciones
    public static double RESTRICTED_LOWER_LIMIT = 10;      // Límite inferior cuando el brazo está bajo
    
    // Nombres para Dashboard
    public static final String DASH_POS = "Elevador/Pos";
    public static final String DASH_TARGET = "Elevador/Target";
    public static final String DASH_RESET_ENCODER = "Elevador/Reset Encoder";
    public static final String DASH_ELEVATOR_WARNING = "Elevador Restricción";

    /////////////////////////////////////////////////////////////
    // CONSTRUCTOR
    /////////////////////////////////////////////////////////////
    public ElevatorSubsystem(ArmSubsystem armSubsystem) {
        this.armSubsystem = armSubsystem;

        // Configuración de motores
        leftMotorLeader = new SparkMax(ElevatorConstants.LEFT_LEADER_CAN_ID, MotorType.kBrushless);
        rightMotorFollow = new SparkMax(ElevatorConstants.RIGHT_FOLLOW_CAN_ID, MotorType.kBrushless);
        
        // Configuración de tiempo de espera CAN
        leftMotorLeader.setCANTimeout(DriveConstants.CAN_TIMEOUT);
        rightMotorFollow.setCANTimeout(DriveConstants.CAN_TIMEOUT);
        
        // Configuración de controlador y motores
        controller = leftMotorLeader.getClosedLoopController();
        leftMotorLeader.configure(getLeaderConfig(), ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
        rightMotorFollow.configure(getFollowConfig(), ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);

        // Configuración del encoder
        elevatorEncoder = leftMotorLeader.getEncoder();
        reset();
        
        // Botón para resetear encoder en Dashboard
        SmartDashboard.putData(DASH_RESET_ENCODER, new InstantCommand(this::reset));
    }

    /////////////////////////////////////////////////////////////
    // CONFIGURACIÓN DE MOTORES
    /////////////////////////////////////////////////////////////
    
    /**
     * Configuración para el motor seguidor
     */
    private SparkBaseConfig getFollowConfig() {
        return new SparkMaxConfig()
            .follow(leftMotorLeader, true)  // Seguir al motor líder con inversión
            .idleMode(SparkBaseConfig.IdleMode.kBrake); // Modo freno al estar inactivo
    }

    /**
     * Configuración para el motor líder
     */
    private static SparkMaxConfig getLeaderConfig() {
        SparkMaxConfig leaderConfig = new SparkMaxConfig();

        // Configuración de control PID
        leaderConfig.closedLoop
            .p(0.02)    // Ganancia proporcional
            .i(0)        // Ganancia integral
            .d(0)        // Ganancia derivativa
            .feedbackSensor(ClosedLoopConfig.FeedbackSensor.kPrimaryEncoder) // Usar encoder primario
            .maxMotion
                .maxVelocity(1.5)      // Velocidad máxima
                .maxAcceleration(0.05) // Aceleración máxima
                .allowedClosedLoopError(0.5); // Error permitido

        // Configuración general del motor
        leaderConfig
            .voltageCompensation(NEOMotorsConstants.VOLTAGE_COMPENSATION) // Compensación de voltaje
            .smartCurrentLimit(NEOMotorsConstants.CURRENT_LIMIT)          // Límite de corriente
            .idleMode(SparkBaseConfig.IdleMode.kBrake)                    // Modo freno
            .inverted(false);                                             // No invertido

        return leaderConfig;
    }

    /////////////////////////////////////////////////////////////
    // MÉTODOS DE LA INTERFAZ MOVABLESUBSYSTEM
    /////////////////////////////////////////////////////////////
    
    @Override
    public double getActualPosition() {
        return elevatorEncoder.getPosition();
    }

    @Override
    public void move(double position) {
        // Aplica límites dinámicos basados en posición del brazo
        double minLimit = isRestrictedArea() ? RESTRICTED_LOWER_LIMIT : LOWER_LIMIT;
        this.position = MathUtil.clamp(position, minLimit, UPPER_LIMIT);
    }

    @Override
    public double getTargetPosition() {
        return position;
    }

    @Override
    public void reset() {
        this.position = 0;
        elevatorEncoder.setPosition(0);
        controller.setReference(this.position, SparkBase.ControlType.kPosition);
    }

    /////////////////////////////////////////////////////////////
    // MÉTODOS PERIÓDICOS
    /////////////////////////////////////////////////////////////
    @Override
    public void periodic() {
        // Actualización de valores en Dashboard
        SmartDashboard.putNumber(DASH_POS, getActualPosition());
        SmartDashboard.putNumber(DASH_TARGET, position);

        // Mostrar advertencia si está en zona restringida durante teleoperado
        if (isRestrictedArea() && RobotState.isTeleop() && RobotState.isEnabled()) {
            SmartDashboard.putString(DASH_ELEVATOR_WARNING, "⚠ Elevador restringido! Muñeca muy baja.");
        }

        // Actualizar referencia de posición del controlador
        controller.setReference(this.position, SparkBase.ControlType.kPosition);
    }

    /////////////////////////////////////////////////////////////
    // MÉTODOS DE SEGURIDAD
    /////////////////////////////////////////////////////////////
    
    /**
     * Verifica si el elevador está en zona restringida
     * @return true si el brazo está por debajo del umbral de seguridad
     */
    public boolean isRestrictedArea() {
        return armSubsystem.getActualPosition() < ARM_THRESHOLD;
    }
}