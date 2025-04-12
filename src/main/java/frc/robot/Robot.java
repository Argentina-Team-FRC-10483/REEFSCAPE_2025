/*
 * Robot.java
 * 
 * Clase principal del robot que implementa TimedRobot.
 * Contiene los métodos llamados automáticamente en cada modo de operación.
 */
package frc.robot;

import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;

/////////////////////////////////////////////////////////////
// CLASE PRINCIPAL DEL ROBOT
/////////////////////////////////////////////////////////////
public class Robot extends TimedRobot {
  // Comando autónomo seleccionado
  private Command m_autonomousCommand;

  // Contenedor principal del robot
  private final RobotContainer m_robotContainer;

  /////////////////////////////////////////////////////////////
  // CONSTRUCTOR - INICIALIZACIÓN
  /////////////////////////////////////////////////////////////
  /**
   * Constructor llamado cuando el robot se inicia por primera vez.
   * Realiza:
   * - Inicialización del RobotContainer
   * - Configuración de bindings de botones
   * - Coloca el selector autónomo en el dashboard
   */
  public Robot() {
    m_robotContainer = new RobotContainer();
  }

  /////////////////////////////////////////////////////////////
  // MÉTODOS DEL CICLO DEL ROBOT
  /////////////////////////////////////////////////////////////

  /**
   * Llamado cada 20ms en todos los modos.
   * Responsable de:
   * - Procesar comandos programados
   * - Ejecutar métodos periódicos de subsistemas
   */
  @Override
  public void robotPeriodic() {
    CommandScheduler.getInstance().run();
  }

  /////////////////////////////////////////////////////////////
  // MÉTODOS DEL MODO DISABLED
  /////////////////////////////////////////////////////////////
  
  /**
   * Llamado una vez al entrar en modo Disabled.
   */
  @Override
  public void disabledInit() {
  }

  /**
   * Llamado periódicamente en modo Disabled.
   */
  @Override
  public void disabledPeriodic() {
  }

  /////////////////////////////////////////////////////////////
  // MÉTODOS DEL MODO AUTÓNOMO
  /////////////////////////////////////////////////////////////

  /**
   * Llamado una vez al iniciar el modo autónomo.
   * Obtiene y programa el comando autónomo seleccionado.
   */
  @Override
  public void autonomousInit() {
    m_autonomousCommand = m_robotContainer.getAutonomousCommand();

    if (m_autonomousCommand != null) {
      m_autonomousCommand.schedule();
    }
  }

  /**
   * Llamado periódicamente durante el modo autónomo.
   */
  @Override
  public void autonomousPeriodic() {
  }

  /////////////////////////////////////////////////////////////
  // MÉTODOS DEL MODO TELEOPERADO
  /////////////////////////////////////////////////////////////

  /**
   * Llamado una vez al iniciar el modo teleoperado.
   * Cancela cualquier comando autónomo en ejecución.
   */
  @Override
  public void teleopInit() {
    if (m_autonomousCommand != null) {
      m_autonomousCommand.cancel();
    }
  }

  /**
   * Llamado periódicamente durante el modo teleoperado.
   */
  @Override
  public void teleopPeriodic() {
  }

  /////////////////////////////////////////////////////////////
  // MÉTODOS DEL MODO TEST
  /////////////////////////////////////////////////////////////

  /**
   * Llamado una vez al iniciar el modo test.
   * Cancela todos los comandos en ejecución.
   */
  @Override
  public void testInit() {
    CommandScheduler.getInstance().cancelAll();
  }

  /**
   * Llamado periódicamente durante el modo test.
   */
  @Override
  public void testPeriodic() {
  }

  /////////////////////////////////////////////////////////////
  // MÉTODOS DE SIMULACIÓN
  /////////////////////////////////////////////////////////////

  /**
   * Llamado una vez al iniciar la simulación.
   */
  @Override
  public void simulationInit() {
  }

  /**
   * Llamado periódicamente durante la simulación.
   */
  @Override
  public void simulationPeriodic() {
  }
}