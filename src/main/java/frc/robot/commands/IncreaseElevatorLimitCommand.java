/*
 * IncreaseElevatorLimitCommand.java
 * 
 * Comando para ajustar temporalmente los límites de movimiento del elevador.
 * Permite extender los límites superiores e inferiores durante operaciones especiales.
 */
package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.ElevatorSubsystem;

/////////////////////////////////////////////////////////////
// COMANDO DE AJUSTE DE LÍMITES DEL ELEVADOR
/////////////////////////////////////////////////////////////
public class IncreaseElevatorLimitCommand extends Command {

    /////////////////////////////////////////////////////////////
    // COMPONENTES Y PARÁMETROS
    /////////////////////////////////////////////////////////////
    private final ElevatorSubsystem elevatorSubsystem; // Subsistema de elevador
    private final double extraUp;     // Extensión adicional para límite superior
    private final double extraDown;   // Extensión adicional para límite inferior

    /////////////////////////////////////////////////////////////
    // CONSTRUCTOR
    /////////////////////////////////////////////////////////////
    /**
     * Crea un nuevo comando para ajustar límites del elevador
     * 
     * @param elevatorSubsystem Subsistema del elevador a modificar
     * @param extraUp Cantidad adicional para el límite superior (valor positivo)
     * @param extraDown Cantidad adicional para el límite inferior (valor positivo)
     */
    public IncreaseElevatorLimitCommand(ElevatorSubsystem elevatorSubsystem, 
                                      double extraUp, 
                                      double extraDown) {
        this.elevatorSubsystem = elevatorSubsystem;
        this.extraUp = extraUp;
        this.extraDown = extraDown;
        
        // Requiere el subsistema para garantizar acceso exclusivo
        addRequirements(elevatorSubsystem);
    }

    /////////////////////////////////////////////////////////////
    // MÉTODOS DEL COMANDO
    /////////////////////////////////////////////////////////////
    
    @Override
    public void initialize() {
        // Ajustar los límites estáticos del elevador
        ElevatorSubsystem.UPPER_LIMIT += extraUp;
        ElevatorSubsystem.LOWER_LIMIT -= extraDown;
        ElevatorSubsystem.RESTRICTED_LOWER_LIMIT -= extraDown;
        
        // Nota: Estos cambios persisten después del comando
        // Considerar implementar revertir en end() si es necesario
    }

    @Override
    public boolean isFinished() {
        // Comando de una sola ejecución
        return true;
    }

    /////////////////////////////////////////////////////////////
    // CONSIDERACIONES DE SEGURIDAD
    /////////////////////////////////////////////////////////////
    /*
     * ADVERTENCIA: 
     * Este comando modifica límites estáticos que afectan a todo el subsistema.
     * Los cambios persisten después de que el comando termina.
     * Para operaciones temporales, considerar:
     */
}