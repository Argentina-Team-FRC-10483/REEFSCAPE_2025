package frc.robot.commands;

import java.util.Optional;

import org.photonvision.EstimatedRobotPose;
import org.photonvision.PhotonCamera;
import org.photonvision.PhotonPoseEstimator;
import org.photonvision.PhotonPoseEstimator.PoseStrategy;
import org.photonvision.targeting.PhotonPipelineResult;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.estimator.DifferentialDrivePoseEstimator;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.kinematics.DifferentialDriveKinematics;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.networktables.StructPublisher;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.EyesSubsystem;
import frc.robot.subsystems.MovementSubsystem;
import frc.robot.util.Gyro;

public class PoseEstimatorCommand extends Command{

    private final EyesSubsystem eyesSubsystem = new EyesSubsystem();
    private final MovementSubsystem movementSubsystem = new MovementSubsystem();

    private final String Pose = "Pose";
    public StructPublisher <Pose2d> posePublisher = NetworkTableInstance.getDefault().getStructTopic(Pose, Pose2d.struct).publish();

    private final PhotonCamera[] cameras = new PhotonCamera[]{new PhotonCamera("WebLaptoop"), new PhotonCamera("Camera_2")};

    private final AprilTagFieldLayout aprilTagFieldLayout = AprilTagFieldLayout.loadField(AprilTagFields.k2025Reefscape);  
    private final Transform3d robotToCam = new Transform3d(new Translation3d(0.5, 0.0, 0.5), new Rotation3d(0,0,0));
    private final PhotonPoseEstimator photonPoseEstimator = new PhotonPoseEstimator(aprilTagFieldLayout, PoseStrategy.MULTI_TAG_PNP_ON_COPROCESSOR, robotToCam);

    private DifferentialDrivePoseEstimator odometry;
    public Optional<Pose2d> estimatedPose;

    final DifferentialDriveKinematics kinematics = new DifferentialDriveKinematics(0.546);
     
    PoseEstimatorCommand(EyesSubsystem ES){
        
    }

    public Optional<EstimatedRobotPose> getEstimatedGlobalPose(PhotonPipelineResult prevEstimatedRobotPose) {
        // photonPoseEstimator.setReferencePose(prevEstimatedRobotPose);
        return photonPoseEstimator.update(prevEstimatedRobotPose);
    }

    @Override
    public void initialize() {

    }
    @Override
    public void execute() {
        odometry = new DifferentialDrivePoseEstimator(
            kinematics,
            Gyro.getInstance().getYawAngle2d(),
            movementSubsystem.getLeftEncoderPosition(),
            movementSubsystem.getRightEncoderPosition(),
            new Pose2d(),
            VecBuilder.fill(0.05, 0.05, Units.degreesToRadians(5)),
            VecBuilder.fill(0.5, 0.5, Units.degreesToRadians(30)));
        for (var camera : cameras){
            var resultCams = camera.getAllUnreadResults();
            for (var result : resultCams){
                estimatedPose = getEstimatedGlobalPose(result).map(x->x.estimatedPose).map(Pose3d::toPose2d);
                if (estimatedPose.isPresent()){
                        odometry.addVisionMeasurement(estimatedPose.get(), result.getTimestampSeconds());
                        posePublisher.accept(estimatedPose.get());
                }else{
                        odometry.addVisionMeasurement(null, eyesSubsystem.Results.getTimestampSeconds());
                }   
            }   
        }
    }
    @Override
  public boolean isFinished() {
    return false;
  }
}