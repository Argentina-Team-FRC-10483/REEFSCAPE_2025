/*
 * IncrementalMoveCommand.java
 * 
 * Comando para mover un subsistema de forma incremental basado en entrada continua.
 * Permite control de posición relativa en lugar de absoluta.
 */
package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.subsystems.MovableSubsystem;
import frc.robot.utils.Utils;
import java.util.function.DoubleSupplier;

/////////////////////////////////////////////////////////////
// COMANDO DE MOVIMIENTO INCREMENTAL
/////////////////////////////////////////////////////////////
public class IncrementalMoveCommand extends Command {

    /////////////////////////////////////////////////////////////
    // COMPONENTES PRINCIPALES
    /////////////////////////////////////////////////////////////
    protected final DoubleSupplier positionChanger; // Proveedor de cambios de posición (-1.0 a 1.0)
    protected final MovableSubsystem subsystem;     // Subsistema movible a controlar

    /////////////////////////////////////////////////////////////
    // CONSTRUCTOR
    /////////////////////////////////////////////////////////////
    /**
     * Crea un nuevo comando de movimiento incremental
     * 
     * @param positionChanger Proveedor de valores para ajuste de posición:
     *                       >0 aumenta posición, <0 disminuye posición
     * @param subsystem Subsistema que implementa MovableSubsystem
     */
    public IncrementalMoveCommand(DoubleSupplier positionChanger, MovableSubsystem subsystem) {
        this.positionChanger = positionChanger;
        this.subsystem = subsystem;
    }

    /////////////////////////////////////////////////////////////
    // MÉTODOS DEL COMANDO
    /////////////////////////////////////////////////////////////
    
    @Override
    public void execute() {
        // 1. Obtener posición objetivo actual
        double targetPosition = subsystem.getTargetPosition();
        
        // 2. Calcular cambio aplicando zona muerta
        double positionChange = Utils.applyDeadZone(
            positionChanger.getAsDouble(), 
            Constants.DeadZone.ELEVATOR
        );
        
        // 3. Actualizar posición objetivo
        targetPosition += positionChange;
        subsystem.move(targetPosition);
    }

    @Override
    public void end(boolean interrupted) {
    }
}