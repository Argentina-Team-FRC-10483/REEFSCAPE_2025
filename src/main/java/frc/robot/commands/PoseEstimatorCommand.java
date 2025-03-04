package frc.robot.commands;

/*import edu.wpi.first.math.estimator.DifferentialDrivePoseEstimator;

public class PoseEstimatorCommand {
    private final DifferentialDrivePoseEstimator m_poseEstimator = new DifferentialDrivePoseEstimator(
        m_kinematics,
        m_gyro.getRotation2d(),
        m_leftEncoder.getDistance(),
        m_rightEncoder.getDistance(),
        new Pose2d(),
        VecBuilder.fill(0.05, 0.05, Units.degreesToRadians(5)),
        VecBuilder.fill(0.5, 0.5, Uniits.degreesToRadians(30)));
        

    m-poseEstimator.updates(
        m_gyro.getRotarion(), m_leftEncoder.getDistance(), m_rightEncoder.getDistance());
        
        Pose3d visionMeasurement3d = objectRobotPose(m_objectInField, m_robotToCamera, m_cameraToObjectEntry);
        
        Pose2d visionMeasurement2d = visionMeasurement3d.toPose2d();
        
        m_poseEstimator.addVisionMeasurement(visionMeasurement2d, Timer.getFPGATimestamp());
    }



*/