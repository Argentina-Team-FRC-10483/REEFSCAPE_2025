/*
 * PositionCamera.java
 * 
 * Clase para manejar la estimación de posición del robot usando cámaras PhotonVision
 * y detección de AprilTags. Proporciona funcionalidad para:
 * - Configuración de cámara PhotonVision
 * - Estimación de pose basada en AprilTags
 * - Publicación de datos de pose en NetworkTables
 */
package frc.robot.utils;

import java.util.List;
import java.util.Optional;
import org.photonvision.EstimatedRobotPose;
import org.photonvision.PhotonCamera;
import org.photonvision.PhotonPoseEstimator;
import org.photonvision.PhotonPoseEstimator.PoseStrategy;
import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.networktables.StructPublisher;

/////////////////////////////////////////////////////////////
// CLASE POSITION CAMERA
/////////////////////////////////////////////////////////////
public class PositionCamera {
    /////////////////////////////////////////////////////////////
    // COMPONENTES PRINCIPALES
    /////////////////////////////////////////////////////////////
    private final PhotonCamera camera;           // Instancia de cámara PhotonVision
    private final Transform3d transform;         // Transformada cámara->robot
    private PhotonPoseEstimator poseEstimator;   // Estimador de posición
    private final StructPublisher<Pose3d> posePublisher; // Publicador de pose en NT

    /////////////////////////////////////////////////////////////
    // CONSTRUCTOR
    /////////////////////////////////////////////////////////////
    /**
     * Crea una nueva instancia de PositionCamera
     * 
     * @param camera    Instancia de PhotonCamera configurada
     * @param transform Transformada desde la cámara al centro del robot
     */
    public PositionCamera(PhotonCamera camera, Transform3d transform) {
        this.camera = camera;
        this.transform = transform;
        
        // Configura el publicador de pose en NetworkTables
        this.posePublisher = NetworkTableInstance.getDefault()
            .getStructTopic("Pose " + camera.getName(), Pose3d.struct)
            .publish();
            
        camera.setDriverMode(true); // Inicia en modo driver (sin procesamiento vision)
    }

    /////////////////////////////////////////////////////////////
    // MÉTODOS PÚBLICOS
    /////////////////////////////////////////////////////////////
    
    /**
     * Inicializa el estimador de pose con el layout de AprilTags
     * 
     * @param layout Layout del campo con posiciones de AprilTags
     */
    public void initPoseEstimator(AprilTagFieldLayout layout) {
        this.poseEstimator = new PhotonPoseEstimator(
            layout, 
            PoseStrategy.MULTI_TAG_PNP_ON_COPROCESSOR, // Usa múltiples tags para mejor precisión
            transform
        );
    }

    /**
     * Actualiza la estimación de posición del robot
     * 
     * @return Optional con la pose estimada si hay detección válida,
     *         o Optional.empty() si no hay detección válida
     */
    public Optional<EstimatedRobotPose> update() {
        // Procesa todos los resultados no leídos de la cámara
        List<EstimatedRobotPose> poses = camera.getAllUnreadResults().stream()
            .map(result -> poseEstimator.update(result))  // Intenta estimar pose
            .filter(Optional::isPresent)                 // Filtra resultados válidos
            .map(Optional::get)                          // Desenvuelve el Optional
            .filter(pose -> pose.targetsUsed.size() >= 2)// Requiere al menos 2 tags
            .toList();

        // Si no hay poses válidas, retorna vacío
        if (poses.isEmpty()) {
            return Optional.empty();
        }

        // Toma la última pose válida y la publica
        var lastPose = poses.get(poses.size() - 1);
        posePublisher.accept(lastPose.estimatedPose);
        
        return Optional.of(lastPose);
    }
}
