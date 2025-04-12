/*
 * AccelIncrementalMoveCommand.java
 * 
 * Comando para movimiento incremental con aceleración controlada.
 * Aplica un filtro de suavizado (SlewRateLimiter) para cambios progresivos de posición.
 * Ideal para control de mecanismos que requieren movimientos suaves.
 */
package frc.robot.commands;

import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.subsystems.MovableSubsystem;
import frc.robot.utils.Utils;
import java.util.function.DoubleSupplier;

/////////////////////////////////////////////////////////////
// COMANDO DE MOVIMIENTO INCREMENTAL CON ACELERACIÓN
/////////////////////////////////////////////////////////////
public class AccelIncrementalMoveCommand extends Command {

    /////////////////////////////////////////////////////////////
    // COMPONENTES PRINCIPALES
    /////////////////////////////////////////////////////////////
    protected final DoubleSupplier positionChanger;  // Proveedor de cambios de posición (-1.0 a 1.0)
    protected final MovableSubsystem subsystem;     // Subsistema movible a controlar
    private final SlewRateLimiter filter;          // Filtro para suavizar cambios

    /////////////////////////////////////////////////////////////
    // CONSTRUCTOR
    /////////////////////////////////////////////////////////////
    /**
     * Crea un nuevo comando de movimiento incremental con aceleración controlada
     * 
     * @param positionChanger Proveedor de valores para ajuste de posición:
     *                       >0 aumenta posición, <0 disminuye posición
     * @param subsystem Subsistema que implementa MovableSubsystem
     * @param speed Tasa máxima de cambio de posición (unidades/segundo)
     */
    public AccelIncrementalMoveCommand(DoubleSupplier positionChanger, 
                                      MovableSubsystem subsystem, 
                                      double speed) {
        this.positionChanger = positionChanger;
        this.subsystem = subsystem;
        this.filter = new SlewRateLimiter(speed);
    }

    /////////////////////////////////////////////////////////////
    // MÉTODOS DEL COMANDO
    /////////////////////////////////////////////////////////////
    
    @Override
    public void execute() {
        // 1. Obtener posición objetivo actual
        double targetPosition = subsystem.getTargetPosition();
        
        // 2. Calcular cambio aplicando:
        //    - Zona muerta (dead zone)
        //    - Filtro de suavizado (SlewRateLimiter)
        double positionChange = filter.calculate(
            Utils.applyDeadZone(
                positionChanger.getAsDouble(), 
                Constants.DeadZone.ELEVATOR
            )
        );
        
        // 3. Actualizar posición objetivo
        targetPosition += positionChange;
        subsystem.move(targetPosition);
    }

    @Override
    public void end(boolean interrupted) {
        // Opcional: Resetear el filtro para el próximo uso
        filter.reset(0);
    }

    /////////////////////////////////////////////////////////////
    // CONSIDERACIONES DE DISEÑO
    /////////////////////////////////////////////////////////////
    /*
     * NOTAS DE IMPLEMENTACIÓN:
     * 1. El filtro SlewRateLimiter suaviza los cambios bruscos de posición
     * 2. La zona muerta (dead zone) ignora pequeñas fluctuaciones del control
     * 3. Para hacer genérica la constante de dead zone:
     *    - Considerar pasarla como parámetro del constructor
     *    - O implementar una interfaz con constantes específicas por subsistema
     */
}