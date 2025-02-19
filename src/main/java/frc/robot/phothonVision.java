package frc.robot;

import java.util.Optional;

import org.photonvision.EstimatedRobotPose;
import org.photonvision.PhotonCamera;
import org.photonvision.PhotonPoseEstimator;
import org.photonvision.PhotonPoseEstimator.PoseStrategy;
import org.photonvision.targeting.PhotonTrackedTarget;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;

public class phothonVision extends Command{

    private final PhotonCamera camera = new PhotonCamera("photonvision");
    private final AprilTagFieldLayout aprilTagFieldLayout = AprilTagFieldLayout.loadField(AprilTagFields.k2025Reefscape);

    Transform3d robotToCam = new Transform3d(new Translation3d(0.5, 0.0, 0.5), new Rotation3d(0,0,0));
    PhotonPoseEstimator photonPoseEstimator = new PhotonPoseEstimator(aprilTagFieldLayout, PoseStrategy.CLOSEST_TO_REFERENCE_POSE, robotToCam);

    public phothonVision(){

    }

    @Override
    public void initialize(){
        SmartDashboard.putBoolean("Deteccion de April", false);
    }
    
    @Override
    public void execute() {
        var result = camera.getLatestResult();

        if (result.getMultiTagResult().isPresent()) {
            Transform3d fieldToCamera = result.getMultiTagResult().get().estimatedPose.best;
            SmartDashboard.putNumber("Camera X:", fieldToCamera.getX());
            SmartDashboard.putNumber("Camera Y:", fieldToCamera.getY());
            SmartDashboard.putNumber("Camera Z:", fieldToCamera.getZ());
        } 

        boolean hasTargets = result.hasTargets();

        SmartDashboard.putBoolean("Deteccion de April", hasTargets);

        PhotonTrackedTarget target = result.getBestTarget();
        if(target != null) {
            double yaw = target.getYaw();
            double pitch = target.getPitch();
            double area = target.getArea();
            int id =  target.fiducialId;
            
            SmartDashboard.putNumber("Yaw", yaw);
            SmartDashboard.putNumber("Pitch", pitch);
            SmartDashboard.putNumber("Area", area);
            SmartDashboard.putNumber("ID", id);
        }
    }

    @Override
    public boolean isFinished() {
        return false;
    }
}
