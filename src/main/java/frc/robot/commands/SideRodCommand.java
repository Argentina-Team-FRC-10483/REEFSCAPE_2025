/*
 * SideRodCommand.java
 * 
 * Comando para controlar los rodillos laterales con suavizado de movimiento.
 * Utiliza un filtro SlewRateLimiter para evitar cambios bruscos de velocidad.
 */
package frc.robot.commands;

import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.RodLateralesSubsystem;

/////////////////////////////////////////////////////////////
// COMANDO DE RODILLOS LATERALES CON SUAVIZADO
/////////////////////////////////////////////////////////////
public class SideRodCommand extends Command {

    /////////////////////////////////////////////////////////////
    // PARÁMETROS DE CONTROL
    /////////////////////////////////////////////////////////////
    private final double power;  // Potencia objetivo (-1.0 a 1.0)
    private final RodLateralesSubsystem rodLateralesSubsystem; // Subsistema controlado

    /////////////////////////////////////////////////////////////
    // FILTRO DE SUAVIZADO
    /////////////////////////////////////////////////////////////
    private final SlewRateLimiter limiter = new SlewRateLimiter(0.5); // Limita la aceleración a 0.5 unidades/seg²

    /////////////////////////////////////////////////////////////
    // CONSTRUCTOR
    /////////////////////////////////////////////////////////////
    /**
     * Crea un nuevo comando para controlar los rodillos laterales
     * 
     * @param rodLateralesSubsystem Subsistema de rodillos a controlar
     * @param power Potencia objetivo del motor (-1.0 a 1.0)
     *             Positivo: rodillo hacia adelante
     *             Negativo: rodillo hacia atrás
     */
    public SideRodCommand(RodLateralesSubsystem rodLateralesSubsystem, double power) {
        this.power = power;
        this.rodLateralesSubsystem = rodLateralesSubsystem;

        // Requiere el subsistema para evitar conflictos de control
        addRequirements(this.rodLateralesSubsystem);
    }

    /////////////////////////////////////////////////////////////
    // MÉTODOS DEL COMANDO
    /////////////////////////////////////////////////////////////
    
    @Override
    public void execute() {
        // Aplica potencia suavizada al subsistema
        double smoothedPower = limiter.calculate(power);
        rodLateralesSubsystem.andarRodillo(smoothedPower);
    }

    @Override
    public void end(boolean isInterrupted) {
        // Detiene los rodillos al finalizar el comando
        rodLateralesSubsystem.andarRodillo(0);
        
        // Reinicia el filtro para el próximo uso
        limiter.reset(0);
    }
}