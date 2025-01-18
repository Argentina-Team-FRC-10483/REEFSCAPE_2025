package frc.robot.commands;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.DebugConstants;
import frc.robot.subsystems.AlgaeIntakeSubsystem;

/**
 * Comando para controlar el subsistema de ingreso de algas (Algae Intake).
 * Este comando utiliza un subsistema para mover un rodillo que opera con una potencia especificada.
 */
public class AlgaeIntakeCommand extends Command {
    // Potencia del motor, los valores positivos y negativos controlan la dirección.
    private final double power; 
    private long debugTimeAlgaeIntakeCommand = 0;
    // Referencia al subsistema encargado del rodillo de ingreso de algas.
    private final AlgaeIntakeSubsystem algaeIntakeSubsystem;

    /**
     * Constructor del comando.
     * algaeIntakeSubsystem: Instancia del subsistema que controla el rodillo.
     * power: Potencia para mover el rodillo (1 es dirección positiva, -1 es dirección negativa).
     */
    public AlgaeIntakeCommand(AlgaeIntakeSubsystem algaeIntakeSubsystem, double power) {
        this.power = power;
        this.algaeIntakeSubsystem = algaeIntakeSubsystem;

        // Registrar el subsistema como requisito del comando para evitar conflictos de recursos.
        addRequirements(this.algaeIntakeSubsystem);
    }

    /**
     * Método llamado una vez cuando el comando comienza.
     */
    @Override
    public void initialize() {
        System.out.println("AlgaeIntakeCommand inicializado. Potencia: " + power);
    }

    /**
     * Método que se llama repetidamente mientras el comando este activo.
     */
    @Override
    public void execute() {
        // Mover el rodillo con la potencia especificada.
        algaeIntakeSubsystem.andarRodillo(power);
        
        // Mensaje de debugging para monitorear la potencia aplicada.
        long currentTime = System.currentTimeMillis();
        if (currentTime - debugTimeAlgaeIntakeCommand >= DebugConstants.DEBUG_INTERVAL_MS) {
          debugTimeAlgaeIntakeCommand = currentTime; // Actualiza el tiempo del último mensaje.
          System.out.println("Ejecutando AlgaeIntakeCommand con potencia: " + power);
    }
    }

    /**
     * Método que se llama una vez cuando el comando termina o es interrumpido.
     * isInterrupted: Indica si el comando fue interrumpido antes de completarse.
     */
    @Override
    public void end(boolean isInterrupted) {
        // Detener el rodillo al finalizar el comando.
        algaeIntakeSubsystem.andarRodillo(0);
        
        // Mensaje de debugging para indicar que el comando ha finalizado.
        System.out.println("AlgaeIntakeCommand finalizado. Interrumpido: " + isInterrupted);
    }

    /**
     * Método que determina si el comando ha terminado.
     * return Siempre retorna false, ya que este comando está diseñado para ejecutarse indefinidamente.
     */
    @Override
    public boolean isFinished() {
        return false;
    }
}
