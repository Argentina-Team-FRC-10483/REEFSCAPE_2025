package frc.robot.utils;

import java.util.List;
import java.util.Optional;
import java.util.function.BiConsumer;

import org.photonvision.EstimatedRobotPose;
import org.photonvision.PhotonCamera;
import org.photonvision.PhotonPoseEstimator;
import org.photonvision.PhotonUtils;
import org.photonvision.PhotonPoseEstimator.PoseStrategy;
import org.photonvision.targeting.PhotonPipelineMetadata;
import org.photonvision.targeting.PhotonPipelineResult;
import org.photonvision.targeting.PhotonTrackedTarget;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.math.estimator.DifferentialDrivePoseEstimator;
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

public class CameraInterFace {

    private final String Pose = "Pose";

    private final List<PositionCamera> cameras;

    StructPublisher<Pose2d> posePublisher = NetworkTableInstance.getDefault().getStructTopic(Pose, Pose2d.struct)
            .publish();

    private final BiConsumer<Pose2d, Double> addVisonConsumer;

    private final AprilTagFieldLayout aprilTagFieldLayout = AprilTagFieldLayout
            .loadField(AprilTagFields.k2025Reefscape);

    public CameraInterFace(List<PositionCamera> inputCameras, BiConsumer<Pose2d, Double> addVisonConsumer) {
        this.cameras = inputCameras;
        this.cameras.forEach(x -> x.initPoseEstimator(aprilTagFieldLayout));
        this.addVisonConsumer = addVisonConsumer;
    }

    public void periodic() {
        cameras.forEach(x->{
            x.update().ifPresent(pose->{
                addVisonConsumer.accept(pose.estimatedPose.toPose2d(), pose.timestampSeconds);
            });;
        });

    }
}
