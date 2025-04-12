/*
 * IncreaseArmLimitCommand.java
 * 
 * Comando para ajustar temporalmente los límites de movimiento del brazo/articulación
 * y sus restricciones relacionadas en el elevador. Permite extender los límites
 * durante operaciones especiales o mantenimiento.
 */
package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.ArmSubsystem;
import frc.robot.subsystems.ElevatorSubsystem;

/////////////////////////////////////////////////////////////
// COMANDO DE AJUSTE DE LÍMITES DEL BRAZO
/////////////////////////////////////////////////////////////
public class IncreaseArmLimitCommand extends Command {

    /////////////////////////////////////////////////////////////
    // COMPONENTES Y PARÁMETROS
    /////////////////////////////////////////////////////////////
    private final ArmSubsystem armSubsystem;  // Subsistema del brazo a modificar
    private final double extraUp;             // Extensión adicional para límite superior (valor positivo)
    private final double extraDown;           // Extensión adicional para límite inferior (valor positivo)

    /////////////////////////////////////////////////////////////
    // CONSTRUCTOR
    /////////////////////////////////////////////////////////////
    /**
     * Crea un nuevo comando para ajustar límites del brazo
     * 
     * @param armSubsystem Subsistema del brazo a modificar
     * @param extraUp Cantidad adicional para el límite superior (valor positivo)
     * @param extraDown Cantidad adicional para el límite inferior (valor positivo)
     */
    public IncreaseArmLimitCommand(ArmSubsystem armSubsystem,
                                 double extraUp, 
                                 double extraDown) {
        this.armSubsystem = armSubsystem;
        this.extraUp = extraUp;
        this.extraDown = extraDown;
        
        // Requiere el subsistema para garantizar acceso exclusivo
        addRequirements(armSubsystem);
    }

    /////////////////////////////////////////////////////////////
    // MÉTODOS DEL COMANDO
    /////////////////////////////////////////////////////////////
    
    @Override
    public void initialize() {
        // Ajustar los límites estáticos del brazo
        ArmSubsystem.UPPER_LIMIT += extraUp;
        ArmSubsystem.LOWER_LIMIT -= extraDown;
        
        // Ajustar el umbral relacionado en el elevador
        ElevatorSubsystem.ARM_THRESHOLD -= extraDown;
        
        // Nota: Estos cambios son persistentes después del comando
    }

    @Override
    public boolean isFinished() {
        // Comando de ejecución única
        return true;
    }

    /////////////////////////////////////////////////////////////
    // CONSIDERACIONES DE SEGURIDAD
    /////////////////////////////////////////////////////////////
    /*
     * ADVERTENCIA IMPORTANTE:
     * 1. Este comando modifica valores estáticos que afectan múltiples subsistemas
     * 2. Los cambios persisten después de que el comando termina
     */
}