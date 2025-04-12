/*
 * Gyro.java
 * 
 * Clase Singleton para manejar el sensor de orientación NavX (AHRS)
 * Proporciona métodos para obtener y visualizar:
 * - Ángulos de orientación (yaw, pitch, roll)
 * - Estado de conexión del dispositivo
 */
package frc.robot.utils;

import com.studica.frc.AHRS;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

/////////////////////////////////////////////////////////////
// CLASE GYRO (SINGLETON)
/////////////////////////////////////////////////////////////
public class Gyro {
    /////////////////////////////////////////////////////////////
    // INSTANCIA SINGLETON Y COMPONENTES
    /////////////////////////////////////////////////////////////
    private static Gyro instance = null;  // Instancia única (Singleton)
    private final AHRS navx;             // Objeto AHRS para el NavX

    /////////////////////////////////////////////////////////////
    // CONSTRUCTOR PRIVADO (SINGLETON)
    /////////////////////////////////////////////////////////////
    /**
     * Constructor privado para patrón Singleton.
     * Inicializa el sensor NavX a través del puerto SPI MXP.
     */
    private Gyro() {
        navx = new AHRS(AHRS.NavXComType.kMXP_SPI);
    }

    /////////////////////////////////////////////////////////////
    // MÉTODOS DE OBTENCIÓN DE ORIENTACIÓN
    /////////////////////////////////////////////////////////////
    
    /**
     * Obtiene el ángulo de guiñada (yaw)
     * @return Ángulo en grados (-180 a 180)
     */
    public double getYawAngle() {
        return -navx.getAngle();  // Invertido para convención estándar
    }

    /**
     * Obtiene el ángulo de alabeo (roll)
     * @return Ángulo en grados
     */
    public double getRollAngle() {
        return navx.getRoll();
    }

    /**
     * Obtiene el ángulo de cabeceo (pitch)
     * @return Ángulo en grados
     */
    public double getPitchAngle() {
        return navx.getPitch();
    }

    /**
     * Obtiene el ángulo actual del robot (alias de getYawAngle)
     * @return Ángulo en grados (-180 a 180)
     */
    public double getRobotAngle() {
        return getYawAngle();
    }

    /////////////////////////////////////////////////////////////
    // MÉTODOS DE ESTADO DEL DISPOSITIVO
    /////////////////////////////////////////////////////////////
    
    /**
     * Verifica si el NavX está conectado
     * @return true si está conectado, false en caso contrario
     */
    public boolean navXConnected() {
        return navx.isConnected();
    }

    /////////////////////////////////////////////////////////////
    // MÉTODOS DE VISUALIZACIÓN
    /////////////////////////////////////////////////////////////
    
    /**
     * Muestra los valores del giroscopio en SmartDashboard
     */
    public void outputValues() {
        SmartDashboard.putNumber("Yaw Angle", getYawAngle());
        SmartDashboard.putNumber("Roll Angle", getRollAngle());
        SmartDashboard.putNumber("Pitch Angle", getPitchAngle());
        SmartDashboard.putNumber("Robot Angle", getRobotAngle());
        SmartDashboard.putBoolean("Gyro Connected", navXConnected());
    }

    /////////////////////////////////////////////////////////////
    // MÉTODO DE ACCESO SINGLETON
    /////////////////////////////////////////////////////////////
    
    /**
     * Obtiene la instancia única del giroscopio
     * @return Instancia Singleton de Gyro
     */
    public static Gyro getInstance() {
        if (instance == null) {
            instance = new Gyro();
        }
        return instance;
    }
}