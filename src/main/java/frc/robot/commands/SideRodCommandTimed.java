/*
 * SideRodCommandTimed.java
 * 
 * Comando que controla los rodillos laterales por un tiempo determinado.
 * Se usa para operaciones temporizadas como expulsar/introducir elementos.
 */
package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.RodLateralesSubsystem;

/////////////////////////////////////////////////////////////
// COMANDO TEMPORIZADO DE RODILLOS LATERALES
/////////////////////////////////////////////////////////////
public class SideRodCommandTimed extends Command {
    
    /////////////////////////////////////////////////////////////
    // PARÁMETROS DE CONTROL
    /////////////////////////////////////////////////////////////
    private final double power;            // Potencia a aplicar (-1.0 a 1.0)
    private final double durationSeconds;  // Duración en segundos
    private final RodLateralesSubsystem rodLateralesSubsystem; // Subsistema controlado
    
    /////////////////////////////////////////////////////////////
    // VARIABLES DE TIEMPO
    /////////////////////////////////////////////////////////////
    private double startTime;              // Tiempo de inicio (ms)

    /////////////////////////////////////////////////////////////
    // CONSTRUCTOR
    /////////////////////////////////////////////////////////////
    /**
     * Crea un nuevo comando temporizado para rodillos laterales
     * 
     * @param rodLateralesSubsystem Subsistema de rodillos a controlar
     * @param power Potencia del motor (-1.0 a 1.0)
     * @param durationSeconds Duración del movimiento en segundos
     */
    public SideRodCommandTimed(RodLateralesSubsystem rodLateralesSubsystem, 
                             double power, 
                             double durationSeconds) {
        this.power = power;
        this.rodLateralesSubsystem = rodLateralesSubsystem;
        this.durationSeconds = durationSeconds;

        addRequirements(this.rodLateralesSubsystem); // Requiere el subsistema
    }

    /////////////////////////////////////////////////////////////
    // MÉTODOS DEL COMANDO
    /////////////////////////////////////////////////////////////
    
    @Override
    public void initialize() {
        // Registra el tiempo de inicio
        startTime = System.currentTimeMillis();
    }

    @Override
    public void execute() {
        // Aplica la potencia al subsistema
        rodLateralesSubsystem.andarRodillo(power);
    }

    @Override
    public boolean isFinished() {
        // Comprueba si ha pasado el tiempo especificado
        return (System.currentTimeMillis() - startTime) > (durationSeconds * 1000);
    }

    @Override
    public void end(boolean isInterrupted) {
        // Detiene los rodillos al finalizar
        rodLateralesSubsystem.andarRodillo(0);
    }
}