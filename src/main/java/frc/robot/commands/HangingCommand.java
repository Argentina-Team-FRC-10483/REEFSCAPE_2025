/*
 * HangingCommand.java
 * 
 * Comando para controlar el sistema de enganche (trepado) del robot.
 * Permite control manual del mecanismo con detención automática al finalizar.
 */
package frc.robot.commands;

import java.util.function.DoubleSupplier;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.HangingSubsystem;

/////////////////////////////////////////////////////////////
// COMANDO DE CONTROL DE ENGANCHE
/////////////////////////////////////////////////////////////
public class HangingCommand extends Command {

    /////////////////////////////////////////////////////////////
    // COMPONENTES PRINCIPALES
    /////////////////////////////////////////////////////////////
    private final HangingSubsystem hangingSubsystem;  // Subsistema de enganche
    private final DoubleSupplier hangingPower;       // Proveedor de potencia (-1.0 a 1.0)

    /////////////////////////////////////////////////////////////
    // CONSTRUCTOR
    /////////////////////////////////////////////////////////////
    /**
     * Crea un nuevo comando de control de enganche
     * 
     * @param hangingSubsystem Subsistema de enganche a controlar
     * @param hangingPower Proveedor de valor de potencia:
     *                    >0: Mueve el enganche hacia arriba
     *                    <0: Mueve el enganche hacia abajo
     *                    0: Detiene el movimiento
     */
    public HangingCommand(HangingSubsystem hangingSubsystem, DoubleSupplier hangingPower) {
        this.hangingSubsystem = hangingSubsystem;
        this.hangingPower = hangingPower;

        // Requiere acceso exclusivo al subsistema
        addRequirements(this.hangingSubsystem);
    }

    /////////////////////////////////////////////////////////////
    // MÉTODOS DEL COMANDO
    /////////////////////////////////////////////////////////////
    
    @Override
    public void execute() {
        // Obtener y aplicar la potencia actualizada
        double power = hangingPower.getAsDouble();
        hangingSubsystem.move(power);
    }

    @Override
    public void end(boolean isInterrupted) {
        // Detener el mecanismo al finalizar el comando
        hangingSubsystem.move(0);
    }

    /////////////////////////////////////////////////////////////
    // CONSIDERACIONES DE SEGURIDAD
    /////////////////////////////////////////////////////////////
    /*
     * Notas importantes:
     * 1. El subsistema HangingSubsystem ya incluye:
     *    - Límites de movimiento (upper/lower limits)
     *    - Zonas de reducción de velocidad (slowdown zones)
     *    - Filtro de suavizado (SlewRateLimiter)
     * 2. El comando garantiza que el mecanismo se detenga cuando:
     *    - El comando es cancelado
     *    - El robot es deshabilitado
     *    - Finaliza el modo de control correspondiente
     */
}