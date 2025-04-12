/*
 * MovableSubsystem.java
 * 
 * Interfaz que define el contrato para subsistemas movibles del robot.
 * Proporciona métodos comunes para control de posición y movimiento.
 */
package frc.robot.subsystems;

/////////////////////////////////////////////////////////////
// INTERFAZ MOVABLE SUBSYSTEM
/////////////////////////////////////////////////////////////
public interface MovableSubsystem {
    
    /////////////////////////////////////////////////////////////
    // MÉTODOS OBLIGATORIOS
    /////////////////////////////////////////////////////////////
    
    /**
     * Reinicia el subsistema a su estado inicial
     * (ej. resetea encoders, posición actual, etc.)
     */
    void reset();

    /**
     * Obtiene la posición actual del subsistema
     * @return Posición actual en unidades del subsistema
     */
    double getActualPosition();

    /**
     * Obtiene la posición objetivo del subsistema
     * @return Posición objetivo en unidades del subsistema
     */
    double getTargetPosition();

    /**
     * Mueve el subsistema a una posición específica
     * @param position Posición deseada en unidades del subsistema
     */
    void move(double position);
}