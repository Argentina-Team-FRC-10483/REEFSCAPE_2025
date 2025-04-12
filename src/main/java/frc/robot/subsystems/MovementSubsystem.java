/*
 * MovementSubsystem.java
 * 
 * Subsistema que controla el movimiento del robot (chasis diferencial)
 * Incluye:
 * - Configuración de motores y encoders
 * - Odometría y seguimiento de posición
 * - Control de movimiento (teleoperado y autónomo)
 * - Integración con visión (PhotonVision)
 */
package frc.robot.subsystems;

import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.config.RobotConfig;
import com.pathplanner.lib.controllers.PPLTVController;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.SparkMaxConfig;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.estimator.DifferentialDrivePoseEstimator;
import edu.wpi.first.math.geometry.*;
import edu.wpi.first.math.kinematics.*;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.networktables.StructPublisher;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.DriveConstants;
import frc.robot.utils.CameraInterFace;
import frc.robot.utils.Gyro;
import frc.robot.utils.PositionCamera;
import org.photonvision.PhotonCamera;
import java.util.List;

/////////////////////////////////////////////////////////////
// SUBSISTEMA DE MOVIMIENTO
/////////////////////////////////////////////////////////////
public class MovementSubsystem extends SubsystemBase {

    /////////////////////////////////////////////////////////////
    // COMPONENTES DEL SUBSISTEMA
    /////////////////////////////////////////////////////////////
    private final SparkMax leftLeader;
    private final SparkMax leftFollow;
    private final SparkMax rightLeader;
    private final SparkMax rightFollow;
    private final RelativeEncoder leftEncoder;
    private final RelativeEncoder rightEncoder;
    private final DifferentialDrive drive;
    private final DifferentialDriveKinematics kinematics = new DifferentialDriveKinematics(0.546);
    private DifferentialDrivePoseEstimator odometry;
    private final CameraInterFace cameraInterFace;
    private Field2d field2d;
    private StructPublisher<Pose2d> posePublisher;

    /////////////////////////////////////////////////////////////
    // CONSTANTES DE CONVERSIÓN
    /////////////////////////////////////////////////////////////
    private static final double kDriveRotToMts = (15.24 * Math.PI * 1.0 / 100.0); // Conversión rotaciones a metros
    private static final double kDriveRPMToMps = (2 * Math.PI * 0.0762) / 60;     // Conversión RPM a m/s

    /////////////////////////////////////////////////////////////
    // CONSTRUCTOR
    /////////////////////////////////////////////////////////////
    public MovementSubsystem() {
        // Configuración de motores
        leftLeader = new SparkMax(DriveConstants.LEFT_LEADER_CAN_ID, MotorType.kBrushed);
        leftFollow = new SparkMax(DriveConstants.LEFT_FOLLOW_CAN_ID, MotorType.kBrushed);
        rightLeader = new SparkMax(DriveConstants.RIGHT_LEADER_CAN_ID, MotorType.kBrushed);
        rightFollow = new SparkMax(DriveConstants.RIGHT_FOLLOW_CAN_ID, MotorType.kBrushed);

        // Configuración del chasis diferencial
        drive = new DifferentialDrive(leftLeader, rightLeader);
        configureMotors();
        
        // Configuración de encoders
        leftEncoder = leftLeader.getEncoder();
        rightEncoder = rightLeader.getEncoder();

        // Configuraciones adicionales
        configureAutoBuilder();
        configureOdometry();
        
        // Configuración del sistema de visión
        cameraInterFace = new CameraInterFace(
            List.of(
                new PositionCamera(
                    new PhotonCamera("Camera_1"),
                    new Transform3d(new Translation3d(-0.356, -0.001, 0.571), new Rotation3d(0, 0, Math.PI))
                ),
                new PositionCamera(
                    new PhotonCamera("Camera_2"),
                    new Transform3d(new Translation3d(0.084, -0.01, 0.971), new Rotation3d(0, Units.degreesToRadians(10), 0))
                ),
                new PositionCamera(
                    new PhotonCamera("Camera_3"),
                    new Transform3d(new Translation3d(-0.356, -0.001, 0.571), new Rotation3d(0, 0, Math.PI))
                )
            ),
            odometry::addVisionMeasurement
        );
    }

    /////////////////////////////////////////////////////////////
    // CONFIGURACIÓN DE MOTORES
    /////////////////////////////////////////////////////////////
    private void configureMotors() {
        SparkMaxConfig config = new SparkMaxConfig();
        config.voltageCompensation(12);
        config.smartCurrentLimit(DriveConstants.DRIVE_MOTOR_CURRENT_LIMIT);

        // Configuración seguidores
        config.follow(leftLeader);
        leftFollow.configure(config, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
        config.follow(rightLeader);
        rightFollow.configure(config, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);

        // Configuración líderes
        config.disableFollowerMode();
        rightLeader.configure(config, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
        config.inverted(true); // Inversión lado izquierdo para movimiento coherente
        leftLeader.configure(config, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
    }

    /////////////////////////////////////////////////////////////
    // CONFIGURACIÓN DE ODOMETRÍA
    /////////////////////////////////////////////////////////////
    private void configureOdometry() {
        odometry = new DifferentialDrivePoseEstimator(
            kinematics,
            Rotation2d.fromDegrees(Gyro.getInstance().getYawAngle()),
            getLeftEncoderPosition(),
            getRightEncoderPosition(),
            new Pose2d()
        );
        
        field2d = new Field2d();
        SmartDashboard.putData("Reset encoders", new InstantCommand(this::resetOdometry));
        posePublisher = NetworkTableInstance.getDefault().getStructTopic("Pose", Pose2d.struct).publish();
        resetOdometry();
    }

    /////////////////////////////////////////////////////////////
    // CONFIGURACIÓN AUTÓNOMA
    /////////////////////////////////////////////////////////////
    private void configureAutoBuilder() {
        try {
            RobotConfig configAuto = RobotConfig.fromGUISettings();
            PPLTVController controller = new PPLTVController(0.02);
            
            AutoBuilder.configure(
                this::getPose,
                this::resetPose,
                this::getRobotRelativeSpeeds,
                (speeds, feedforwards) -> driveRobotRelative(speeds),
                controller,
                configAuto,
                () -> DriverStation.getAlliance().filter(value -> value == DriverStation.Alliance.Blue).isPresent(),
                this
            );
        } catch (Exception e) {
            e.printStackTrace();
        }
    }

    /////////////////////////////////////////////////////////////
    // MÉTODOS PERIÓDICOS
    /////////////////////////////////////////////////////////////
    @Override
    public void periodic() {
        // Actualización de odometría
        odometry.update(
            Rotation2d.fromDegrees(Gyro.getInstance().getYawAngle()),
            getLeftEncoderPosition(),
            getRightEncoderPosition()
        );
        
        // Visualización de datos
        field2d.setRobotPose(getPose());
        posePublisher.accept(getPose());
        cameraInterFace.periodic();
        Gyro.getInstance().outputValues();
        SmartDashboard.putData(drive);
    }

    /////////////////////////////////////////////////////////////
    // MÉTODOS DE OBTENCIÓN DE DATOS
    /////////////////////////////////////////////////////////////
    public double getLeftEncoderPosition() {
        return leftEncoder.getPosition() * kDriveRotToMts;
    }

    public double getRightEncoderPosition() {
        return -rightEncoder.getPosition() * kDriveRotToMts;
    }

    public Pose2d getPose() {
        return odometry.getEstimatedPosition();
    }

    public ChassisSpeeds getRobotRelativeSpeeds() {
        return kinematics.toChassisSpeeds(
            new DifferentialDriveWheelSpeeds(
                leftEncoder.getVelocity() * kDriveRPMToMps,
                -rightEncoder.getVelocity() * kDriveRPMToMps
            )
        );
    }

    /////////////////////////////////////////////////////////////
    // MÉTODOS DE CONTROL DE MOVIMIENTO
    /////////////////////////////////////////////////////////////
    public void driveRobotRelative(ChassisSpeeds speeds) {
        var wheelSpeeds = kinematics.toWheelSpeeds(speeds);
        drive.tankDrive(wheelSpeeds.leftMetersPerSecond, wheelSpeeds.rightMetersPerSecond, false);
    }

    public void driveArcade(double xSpeed, double zRotation) {
        drive.arcadeDrive(xSpeed, zRotation, false);
    }

    public void resetPose(Pose2d pose2d) {
        odometry.resetPose(pose2d);
    }

    private void resetOdometry() {
        leftEncoder.setPosition(0);
        rightEncoder.setPosition(0);
        odometry.resetPose(new Pose2d(0, 0, new Rotation2d(0)));
    }
}