/*
 * MoveToPositionCommand.java
 * 
 * Comando genérico para mover cualquier subsistema MovableSubsystem
 * a una posición específica con tolerancia configurable.
 * Puede operar en modo bloqueante (espera completar) o no bloqueante.
 */
package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.MovableSubsystem;

/////////////////////////////////////////////////////////////
// COMANDO DE MOVIMIENTO A POSICIÓN
/////////////////////////////////////////////////////////////
public class MoveToPositionCommand extends Command {

    /////////////////////////////////////////////////////////////
    // PARÁMETROS DE CONFIGURACIÓN
    /////////////////////////////////////////////////////////////
    protected final MovableSubsystem subsystem;  // Subsistema a controlar
    final double targetPosition;                 // Posición objetivo deseada
    final double tolerance;                      // Margen de error aceptable
    private final boolean waitForCompletion;     // Si debe esperar alcanzar la posición

    /////////////////////////////////////////////////////////////
    // CONSTRUCTOR
    /////////////////////////////////////////////////////////////
    /**
     * Crea un nuevo comando de movimiento a posición
     * 
     * @param targetPosition Posición objetivo deseada
     * @param subsystem Subsistema que implementa MovableSubsystem
     * @param tolerance Margen de error aceptable (unidades del subsistema)
     * @param waitForCompletion True = espera alcanzar la posición, False = comando no bloqueante
     */
    public MoveToPositionCommand(double targetPosition, MovableSubsystem subsystem, 
                               double tolerance, boolean waitForCompletion) {
        this.targetPosition = targetPosition;
        this.subsystem = subsystem;
        this.tolerance = tolerance;
        this.waitForCompletion = waitForCompletion;
    }

    /////////////////////////////////////////////////////////////
    // MÉTODOS DEL COMANDO
    /////////////////////////////////////////////////////////////
    
    @Override
    public void execute() {
        // Envía la posición objetivo al subsistema
        subsystem.move(targetPosition);
    }

    @Override
    public boolean isFinished() {
        // Para modo no bloqueante o cuando se alcanza la posición
        return !waitForCompletion || 
               Math.abs(subsystem.getActualPosition() - targetPosition) < tolerance;
    }

    @Override
    public void end(boolean interrupted) {
        // Opcional: Podría resetear la posición objetivo si es necesario
        // subsystem.move(0);
    }
}