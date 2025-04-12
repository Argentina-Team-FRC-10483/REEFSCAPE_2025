/*
 * Utils.java
 * 
 * Clase de utilidades con funciones matemáticas y de control comunes
 * para el robot.
 */
package frc.robot.utils;

/////////////////////////////////////////////////////////////
// CLASE DE UTILIDADES
/////////////////////////////////////////////////////////////
public class Utils {

    /////////////////////////////////////////////////////////////
    // MÉTODO PARA APLICAR ZONA MUERTA (DEADZONE)
    /////////////////////////////////////////////////////////////
    
    /**
     * Aplica una zona muerta (deadzone) a un valor de entrada.
     * 
     * @param input    Valor de entrada al que aplicar la zona muerta
     * @param deadZone Límite de la zona muerta (valor absoluto)
     * @return         Valor ajustado (0 si está dentro de la zona muerta, 
     *                 el valor original si está fuera)
     * 
     * Ejemplo de uso:
     * applyDeadZone(0.08, 0.1) -> retorna 0
     * applyDeadZone(0.15, 0.1) -> retorna 0.15
     */
    public static double applyDeadZone(double input, double deadZone) {
        return Math.abs(input) < deadZone ? 0 : input;
    }
}