package frc.robot.subsystems;

import java.util.Optional;

import org.photonvision.EstimatedRobotPose;
import org.photonvision.PhotonCamera;
import org.photonvision.PhotonPoseEstimator;
import org.photonvision.PhotonPoseEstimator.PoseStrategy;
import org.photonvision.targeting.PhotonPipelineMetadata;
import org.photonvision.targeting.PhotonPipelineResult;
import org.photonvision.targeting.PhotonTrackedTarget;

import edu.wpi.first.apriltag.AprilTag;
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
import edu.wpi.first.wpilibj.Timer;



public class EyesSubsystem extends SubsystemBase{

    private final String Pose = "Pose";

    private final PhotonCamera[] cameras = new PhotonCamera[]{new PhotonCamera("Camera_3"), new PhotonCamera("Camera_4")};

    StructPublisher <Pose2d> posePublisher = NetworkTableInstance.getDefault().getStructTopic(Pose, Pose2d.struct).publish();

    private final AprilTagFieldLayout aprilTagFieldLayout = AprilTagFieldLayout.loadField(AprilTagFields.k2025Reefscape);
        
    private final Transform3d robotToCam = new Transform3d(new Translation3d(0.5, 0.0, 0.5), new Rotation3d(0,0,0));
    private final PhotonPoseEstimator photonPoseEstimator = new PhotonPoseEstimator(aprilTagFieldLayout, PoseStrategy.MULTI_TAG_PNP_ON_COPROCESSOR, robotToCam);
    private String number = "";

    public void periodic(){ 
        double start = Timer.getFPGATimestamp();
        for (int i = 0; i < cameras.length; i++){
            PhotonCamera camera = cameras[i];
            var resultCams = camera.getAllUnreadResults();
            for (var result : resultCams){
                number = Integer.toString(i+1);
                PhotonTrackedTarget target = result.getBestTarget();
                SmartDashboard.putBoolean("Deteccion April Cam " + number, result.hasTargets());
                Optional<Pose2d> estimatedPose = getEstimatedGlobalPose(result).map(x->x.estimatedPose).map(Pose3d::toPose2d);
                if (result.getMultiTagResult().isPresent()) {
                    Transform3d fieldToCamera = result.getMultiTagResult().get().estimatedPose.best;
                    SmartDashboard.putNumber("Robot  X:" +number, fieldToCamera.getX());
                    SmartDashboard.putNumber("Robot  Y:" +number, fieldToCamera.getY());
                    SmartDashboard.putNumber("Robot  Z:" +number, fieldToCamera.getZ());
                    SmartDashboard.putNumber("Time:", result.getTimestampSeconds());
                    if (estimatedPose.isPresent()){
                        posePublisher.accept(estimatedPose.get());
                    }else{
                        posePublisher.accept(new Pose2d(new Translation2d(0.5, 0.5), new Rotation2d(0.5)));
                    }
                }else if(result.hasTargets()){
                    var infoFieldTags = aprilTagFieldLayout.getTagPose(resultCams.get(i).getBestTarget().getFiducialId()).get();
                    if (resultCams.get(0).hasTargets() && resultCams.get(0).hasTargets()){
                        double distance = Math.round(Math.pow(infoFieldTags.getX(), 2)+Math.pow(infoFieldTags.getY(), 2));
                        SmartDashboard.putNumber("Camera distancia", distance);
                     }
                    if(resultCams.get(0).hasTargets()){
                        SmartDashboard.putNumber("Camera X" +number, infoFieldTags.getX());
                        SmartDashboard.putNumber("Camera Y" +number, infoFieldTags.getY());
                        SmartDashboard.putNumber("Camera Z" +number, infoFieldTags.getZ());
                        SmartDashboard.putNumber("Camera R" +number, Math.toDegrees(infoFieldTags.getRotation().getZ()));
                    }else if (resultCams.get(1).hasTargets() && target != null){
                        SmartDashboard.putNumber("Camera X" +number, infoFieldTags.getX());
                        SmartDashboard.putNumber("Camera Y" +number, infoFieldTags.getY());
                        SmartDashboard.putNumber("Camera Z" +number, infoFieldTags.getZ());
                    }
                }
                if(target != null) {    
                    double yaw = target.getYaw();
                    double pitch = target.getPitch();
                    double area = target.getArea();
                    double skew = target.getSkew();
                    int id =  target.fiducialId;

                    SmartDashboard.putNumber("Yaw" +number, yaw);
                    SmartDashboard.putNumber("Camara_1 ID", id);                        
                }
            } 
        }
        SmartDashboard.putNumber("Time elapsed", start - Timer.getFPGATimestamp());
    }
    public Optional<EstimatedRobotPose> getEstimatedGlobalPose(PhotonPipelineResult prevEstimatedRobotPose) {
        // photonPoseEstimator.setReferencePose(prevEstimatedRobotPose);
        return photonPoseEstimator.update(prevEstimatedRobotPose);
    }
 }