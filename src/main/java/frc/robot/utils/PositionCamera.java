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

public class PositionCamera {
  final PhotonCamera camera;
  final Transform3d transform;
  PhotonPoseEstimator poseEstimator;
  final StructPublisher<Pose3d> posePublisher;

  public PositionCamera(PhotonCamera camera, Transform3d transform) {
    this.camera = camera;
    this.transform = transform;
    this.posePublisher = NetworkTableInstance
      .getDefault()
      .getStructTopic("Pose " + camera.getName(), Pose3d.struct)
      .publish();
    camera.setDriverMode(true);
  }

  public void initPoseEstimator(AprilTagFieldLayout layout) {
    this.poseEstimator = new PhotonPoseEstimator(layout, PoseStrategy.MULTI_TAG_PNP_ON_COPROCESSOR, transform);
  }

  public Optional<EstimatedRobotPose> update() {
    List<EstimatedRobotPose> poses = camera
      .getAllUnreadResults().stream()
      .map(x -> poseEstimator.update(x))
      .filter(Optional::isPresent)
      .map(Optional::get)
      .filter(pose -> pose.targetsUsed.size() >= 2) // Ensure at least 2 tags are used
      .toList();
    if (poses.isEmpty())
      return Optional.empty();
    var lastPose = poses.get(poses.size() - 1);
    posePublisher.accept(lastPose.estimatedPose);
    return Optional.of(lastPose);
  }
}
