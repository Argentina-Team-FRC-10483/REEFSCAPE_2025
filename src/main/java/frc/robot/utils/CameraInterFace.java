/*
 * CameraInterface.java
 * 
 * Clase para gestionar múltiples cámaras de visión y procesar sus estimaciones de pose.
 * Se encarga de:
 * - Inicializar cámaras con el layout de AprilTags
 * - Procesar actualizaciones periódicas de las cámaras
 * - Comunicar las estimaciones de pose a un consumidor externo
 */
package frc.robot.utils;

import java.util.List;
import java.util.function.BiConsumer;
import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.math.geometry.Pose2d;

/////////////////////////////////////////////////////////////
// CLASE CAMERA INTERFACE
/////////////////////////////////////////////////////////////
public class CameraInterFace {
    
    /////////////////////////////////////////////////////////////
    // COMPONENTES PRINCIPALES
    /////////////////////////////////////////////////////////////
    private final List<PositionCamera> cameras;  // Lista de cámaras configuradas
    private final BiConsumer<Pose2d, Double> addVisonConsumer; // Consumidor de estimaciones de pose
    private final AprilTagFieldLayout aprilTagFieldLayout; // Layout de AprilTags del campo

    /////////////////////////////////////////////////////////////
    // CONSTRUCTOR
    /////////////////////////////////////////////////////////////
    /**
     * Crea una nueva interfaz para gestionar múltiples cámaras de visión
     * 
     * @param inputCameras Lista de cámaras configuradas
     * @param addVisonConsumer Consumidor que recibirá las estimaciones de pose (Pose2d + timestamp)
     */
    public CameraInterFace(List<PositionCamera> inputCameras, BiConsumer<Pose2d, Double> addVisonConsumer) {
        this.cameras = inputCameras;
        this.addVisonConsumer = addVisonConsumer;
        
        // Carga el layout de AprilTags para la temporada 2025 Reefscape
        this.aprilTagFieldLayout = AprilTagFieldLayout.loadField(AprilTagFields.k2025Reefscape);
        
        // Inicializa todas las cámaras con el layout de AprilTags
        this.cameras.forEach(x -> x.initPoseEstimator(aprilTagFieldLayout));
    }

    /////////////////////////////////////////////////////////////
    // MÉTODO PERIÓDICO
    /////////////////////////////////////////////////////////////
    /**
     * Método que debe ser llamado periódicamente para:
     * - Actualizar las estimaciones de todas las cámaras
     * - Enviar las estimaciones válidas al consumidor
     */
    public void periodic() {
        cameras.forEach(camera -> {
            camera.update().ifPresent(pose -> {
                // Convierte la pose 3D a 2D y la envía al consumidor con el timestamp
                addVisonConsumer.accept(
                    pose.estimatedPose.toPose2d(),
                    pose.timestampSeconds
                );
            });
        });
    }
}