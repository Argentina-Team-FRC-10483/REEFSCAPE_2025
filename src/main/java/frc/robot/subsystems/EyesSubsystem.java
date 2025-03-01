package frc.robot.subsystems;

import java.util.Optional;

import org.photonvision.EstimatedRobotPose;
import org.photonvision.PhotonCamera;
import org.photonvision.PhotonPoseEstimator;
import org.photonvision.PhotonPoseEstimator.PoseStrategy;
import org.photonvision.targeting.PhotonPipelineMetadata;
import org.photonvision.targeting.PhotonPipelineResult;
import org.photonvision.targeting.PhotonTrackedTarget;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.networktables.StructPublisher;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class EyesSubsystem extends SubsystemBase{

    private final String Pose = "Pose";

    private final PhotonCamera[] cameras = new PhotonCamera[]{new PhotonCamera("WebLaptoop"), new PhotonCamera("Camera_2")};

    StructPublisher <Pose2d> posePublisher = NetworkTableInstance.getDefault().getStructTopic(Pose, Pose2d.struct).publish();

    private final AprilTagFieldLayout aprilTagFieldLayout = AprilTagFieldLayout.loadField(AprilTagFields.k2025Reefscape);
        
    private final Transform3d robotToCam = new Transform3d(new Translation3d(0.5, 0.0, 0.5), new Rotation3d(0,0,0));
    private final PhotonPoseEstimator photonPoseEstimator = new PhotonPoseEstimator(aprilTagFieldLayout, PoseStrategy.MULTI_TAG_PNP_ON_COPROCESSOR, robotToCam);
    
    public void periodic(){
        for (var camera : cameras){
            var resultCam1 = camera.getAllUnreadResults();
            for (var result : resultCam1){
                if (result.getMultiTagResult().isPresent()) {
                    Transform3d fieldToCamera = result.getMultiTagResult().get().estimatedPose.best;
                    SmartDashboard.putNumber("Camera_1 X:", fieldToCamera.getX());
                    SmartDashboard.putNumber("Camera_1 Y:", fieldToCamera.getY());
                    SmartDashboard.putNumber("Camera_1 Z:", fieldToCamera.getZ());
                    SmartDashboard.putNumber("Time:", result.getTimestampSeconds());
                    Optional<Pose2d> estimatedPose = getEstimatedGlobalPose(result).map(x->x.estimatedPose).map(Pose3d::toPose2d);
                    if (estimatedPose.isPresent()){
                        posePublisher.accept(estimatedPose.get());
                    }else{
                        posePublisher.accept(new Pose2d(new Translation2d(0.5, 0.5), new Rotation2d(0.5)));
                    }
                } 
    
                boolean hasTargets = result.hasTargets();
    
            SmartDashboard.putBoolean("Deteccion de Camara_1 April", hasTargets);
    
            PhotonTrackedTarget target = result.getBestTarget();
                if(target != null) {
                    double yaw = target.getYaw();
                    double pitch = target.getPitch();
                    double area = target.getArea();
                    int id =  target.fiducialId;
                    
                    SmartDashboard.putNumber("Camara_1 Yaw", yaw);
                    SmartDashboard.putNumber("Camara_1 Pitch", pitch);
                    SmartDashboard.putNumber("Camara_1 Area", area);
                    SmartDashboard.putNumber("Camara_1 ID", id);
                }
            }
        }
    }
    
    public Optional<EstimatedRobotPose> getEstimatedGlobalPose(PhotonPipelineResult prevEstimatedRobotPose) {
        // photonPoseEstimator.setReferencePose(prevEstimatedRobotPose);
        return photonPoseEstimator.update(prevEstimatedRobotPose);
    }
}