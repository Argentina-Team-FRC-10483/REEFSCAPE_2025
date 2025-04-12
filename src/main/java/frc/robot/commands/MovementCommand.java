/*
 * MovementCommand.java
 * 
 * Comando para controlar el movimiento del chasis con:
 * - Control de aceleración progresiva
 * - Límites de velocidad configurable
 * - Zonas muertas (dead zones)
 * - Aceleración adicional con botón bumper
 */
package frc.robot.commands;

import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.DeadZone;
import frc.robot.Constants.DriveConstants;
import frc.robot.subsystems.MovementSubsystem;
import frc.robot.utils.Utils;
import java.util.function.BooleanSupplier;
import java.util.function.DoubleSupplier;

/////////////////////////////////////////////////////////////
// COMANDO DE MOVIMIENTO DEL CHASIS
/////////////////////////////////////////////////////////////
public class MovementCommand extends Command {

    /////////////////////////////////////////////////////////////
    // FUENTES DE ENTRADA
    /////////////////////////////////////////////////////////////
    private final DoubleSupplier xSpeed;       // Control de velocidad adelante/atrás
    private final DoubleSupplier zRotation;    // Control de rotación
    private final BooleanSupplier bumper;     // Botón de aceleración
    private final MovementSubsystem driveSubsystem; // Subsistema de movimiento

    /////////////////////////////////////////////////////////////
    // CONTROL DE ACELERACIÓN
    /////////////////////////////////////////////////////////////
    private final SlewRateLimiter filter = new SlewRateLimiter(2); // Filtro para suavizar cambios
    private double lastTime;                  // Último tiempo registrado
    private double accel;                     // Nivel actual de aceleración

    /////////////////////////////////////////////////////////////
    // CONSTRUCTOR
    /////////////////////////////////////////////////////////////
    /**
     * Crea un nuevo comando de movimiento
     * 
     * @param xSpeed Proveedor de velocidad lineal (-1.0 a 1.0)
     * @param bumper Proveedor de estado del botón bumper (aceleración)
     * @param zRotation Proveedor de velocidad angular (-1.0 a 1.0) 
     * @param driveSubsystem Subsistema de movimiento a controlar
     */
    public MovementCommand(
        DoubleSupplier xSpeed,
        BooleanSupplier bumper,
        DoubleSupplier zRotation,
        MovementSubsystem driveSubsystem
    ) {
        this.xSpeed = xSpeed;
        this.bumper = bumper;
        this.zRotation = zRotation;
        this.driveSubsystem = driveSubsystem;
        this.lastTime = Timer.getFPGATimestamp();

        addRequirements(this.driveSubsystem);
    }

    /////////////////////////////////////////////////////////////
    // MÉTODOS DEL COMANDO
    /////////////////////////////////////////////////////////////
    
    @Override
    public void execute() {
        // 1. Obtener valores actuales de los controles
        double speed = xSpeed.getAsDouble();
        double rotation = zRotation.getAsDouble();
        double deltaTime = Timer.getFPGATimestamp() - lastTime;

        // 2. Calcular estado actual de límites
        boolean velocityNotAtLimit = Math.abs(speed) < DriveConstants.AXIS_SPEED_LIMIT;
        boolean rotationNotAtLimit = Math.abs(rotation) < DriveConstants.AXIS_TURN_LIMIT;
        boolean acceleratorPressed = bumper.getAsBoolean();
        boolean accelNotAtLimit = this.accel < DriveConstants.BUMPER_ACCEL_LIMIT;

        // 3. Manejar aceleración progresiva
        if (velocityNotAtLimit && rotationNotAtLimit) {
            // Resetear aceleración cuando los controles están en reposo
            this.accel = DriveConstants.DEAD_POINT;
        } else if (acceleratorPressed && accelNotAtLimit) {
            // Incrementar aceleración mientras se presiona el bumper
            this.accel += DriveConstants.ACCEL_INCREASE * deltaTime;
            this.accel = Math.min(this.accel, DriveConstants.BUMPER_ACCEL_LIMIT);
        } else if (!acceleratorPressed && this.accel > DriveConstants.DEAD_POINT) {
            // Reducir aceleración gradualmente al soltar el bumper
            this.accel -= DriveConstants.ACCEL_INCREASE * deltaTime;
            this.accel = Math.max(this.accel, DriveConstants.DEAD_POINT);
        }

        // 4. Aplicar zonas muertas
        speed = Utils.applyDeadZone(speed, DeadZone.MOVEMENT);
        rotation = Utils.applyDeadZone(rotation, DeadZone.MOVEMENT);

        // 5. Aplicar aceleración y filtro de suavizado
        if (speed > DriveConstants.DEAD_POINT) {
            speed = filter.calculate(speed + accel);
        } else if (speed <= DriveConstants.AXIS_SPEED_LIMIT) {
            speed = filter.calculate(speed - accel);
        }

        // 6. Actualizar tiempo y enviar comandos al chasis
        this.lastTime = Timer.getFPGATimestamp();
        driveSubsystem.driveArcade(speed, rotation);
    }

    @Override
    public void end(boolean interrupted) {
        // Detener el chasis al finalizar el comando
        driveSubsystem.driveArcade(0, 0);
    }
}