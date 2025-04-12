/*
 * Constants.java
 * 
 * Contiene todas las constantes configurables del robot agrupadas por subsistemas.
 * Valores como IDs CAN, límites de corriente, posiciones y configuraciones.
 */
package frc.robot;

/////////////////////////////////////////////////////////////
// CLASE PRINCIPAL DE CONSTANTES
/////////////////////////////////////////////////////////////
public final class Constants {

    /////////////////////////////////////////////////////////////
    // CONSTANTES DEL CHASIS/DRIVETRAIN
    /////////////////////////////////////////////////////////////
    public static final class DriveConstants {
        // Configuración de IDs CAN
        public static final int LEFT_LEADER_CAN_ID = 1;
        public static final int LEFT_FOLLOW_CAN_ID = 2;
        public static final int RIGHT_LEADER_CAN_ID = 3;
        public static final int RIGHT_FOLLOW_CAN_ID = 4;

        // Configuración de control
        public static final int DRIVE_MOTOR_CURRENT_LIMIT = 60;
        public static final double AXIS_SPEED_LIMIT = 0.5;
        public static final double AXIS_TURN_LIMIT = 0.5;
        public static final double BUMPER_ACCEL_LIMIT = 0.5;
        public static final double ACCEL_INCREASE = 0.45;
        public static final double DEAD_POINT = 0.0;

        // Configuración CAN
        public static final int CAN_TIMEOUT = 250;
    }

    /////////////////////////////////////////////////////////////
    // CONSTANTES DE CONTROLADORES
    /////////////////////////////////////////////////////////////
    public static final class OperatorConstants {
        public static final int DRIVER_CONTROLLER_PORT = 0;
        public static final int OPERATOR_CONTROLLER_PORT = 1;
    }

    /////////////////////////////////////////////////////////////
    // CONSTANTES DEL SISTEMA DE ENGANCHE (HANGING)
    /////////////////////////////////////////////////////////////
    public static final class HangingConstants {
        public static final int CAN_ID = 7;
    }

    /////////////////////////////////////////////////////////////
    // CONSTANTES DEL ELEVADOR
    /////////////////////////////////////////////////////////////
    public static final class ElevatorConstants {
        // Configuración de motores
        public static final int LEFT_LEADER_CAN_ID = 5;
        public static final int RIGHT_FOLLOW_CAN_ID = 6;

        // Posiciones predefinidas (setpoints)
        public static final int L0 = 0;       // Posición base/reposo
        public static final double L1 = 16.3; // Nivel 1
        public static final double L2 = 41.1; // Nivel 2
        public static final int L3 = 86;      // Nivel 3 (máximo)

        // Habilitación de posiciones predefinidas
        public static final boolean ENABLE_ELEVATOR_SETPOINTS = true;
    }

    /////////////////////////////////////////////////////////////
    // CONSTANTES GENERALES DE MOTORES NEO
    /////////////////////////////////////////////////////////////
    public static final class NEOMotorsConstants {
        public static final int CURRENT_LIMIT = 40;
        public static final double VOLTAGE_COMPENSATION = 12.0;
    }

    /////////////////////////////////////////////////////////////
    // CONSTANTES DE ZONAS MUERTAS (DEADZONES)
    /////////////////////////////////////////////////////////////
    public static final class DeadZone {
        public static final double ELEVATOR = 0.1;   // Deadzone para el elevador
        public static final double MOVEMENT = 0.1;    // Deadzone para el movimiento
    }

    /////////////////////////////////////////////////////////////
    // CONSTANTES DEL BRAZO
    /////////////////////////////////////////////////////////////
    public static final class ArmConstants {
        public static final int CAN_ID = 9;
    }

    /////////////////////////////////////////////////////////////
    // CONSTANTES DE LA GARRA
    /////////////////////////////////////////////////////////////
    public static final class HandConstants {
        public static final int SIDE_CAN_ID = 11;  // ID CAN para motores laterales de la garra
    }
}
