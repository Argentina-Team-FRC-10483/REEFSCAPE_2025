package frc.robot.utils;

import java.util.List;
import java.util.function.BiConsumer;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.math.geometry.Pose2d;

public class CameraInterFace {
  private final List<PositionCamera> cameras;
  private final BiConsumer<Pose2d, Double> addVisonConsumer;
  private final AprilTagFieldLayout aprilTagFieldLayout = AprilTagFieldLayout
    .loadField(AprilTagFields.k2025Reefscape);

  public CameraInterFace(List<PositionCamera> inputCameras, BiConsumer<Pose2d, Double> addVisonConsumer) {
    this.cameras = inputCameras;
    this.cameras.forEach(x -> x.initPoseEstimator(aprilTagFieldLayout));
    this.addVisonConsumer = addVisonConsumer;
  }

  public void periodic() {
    cameras.forEach(
      x -> x.update().ifPresent(
        pose -> addVisonConsumer.accept(pose.estimatedPose.toPose2d(), pose.timestampSeconds)
      )
    );
  }
}
