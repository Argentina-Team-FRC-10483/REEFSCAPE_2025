package frc.robot.subsystems;

import com.revrobotics.sim.SparkMaxSim;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.SparkBaseConfig;
import com.revrobotics.spark.config.SparkMaxConfig;

import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.DifferentialDriveOdometry;
import edu.wpi.first.math.kinematics.DifferentialDriveWheelPositions;
import edu.wpi.first.math.kinematics.Odometry;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.networktables.DoublePublisher;
import edu.wpi.first.networktables.FloatPublisher;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.networktables.StructPublisher;
import edu.wpi.first.util.struct.StructGenerator;
import edu.wpi.first.wpilibj.AnalogGyro;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.wpilibj.simulation.AnalogGyroSim;
import edu.wpi.first.wpilibj.simulation.DCMotorSim;
import edu.wpi.first.wpilibj.simulation.DifferentialDrivetrainSim;
import edu.wpi.first.wpilibj.simulation.REVPHSim;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.DriveConstants;

public class MovementSubsystem extends SubsystemBase {
    private final DCMotor leftGearbox = DCMotor.getNEO(2);
    private final DCMotor rightGearbox = DCMotor.getNEO(2);
    private final SparkMax leftMain = new SparkMax(DriveConstants.MAIN_LEFT_ID, MotorType.kBrushed);
    private final SparkMax leftFollow = new SparkMax(DriveConstants.FOLLOW_LEFT_ID, MotorType.kBrushed);
    private final SparkMax rightMain = new SparkMax(DriveConstants.MAIN_RIGHT_ID, MotorType.kBrushed);
    private final SparkMax rightFollow = new SparkMax(DriveConstants.FOLLOW_RIGHT_ID, MotorType.kBrushed);
    private SparkMaxSim leftMainSim = new SparkMaxSim(leftMain, leftGearbox);
    private SparkMaxSim rightMainSim = new SparkMaxSim(rightMain, rightGearbox);
    private AnalogGyro gyro = new AnalogGyro(0); // TODO: Delete me
    private AnalogGyroSim gyroSim = new AnalogGyroSim(gyro);
    private final DifferentialDrive drive = new DifferentialDrive(leftMain, rightMain);
    private final Odometry<DifferentialDriveWheelPositions> odometry;
    private final DifferentialDrivetrainSim drivetrainSim = new DifferentialDrivetrainSim(
            leftGearbox, 1, 1, 1, 0.05, 0.9,
            null
    );


    StructPublisher<Pose2d> posePublisher = NetworkTableInstance.getDefault().getStructTopic("CurrentPose", Pose2d.struct).publish();
    StructPublisher<Pose2d> posePublisher2 = NetworkTableInstance.getDefault().getStructTopic("CurrentPose2", Pose2d.struct).publish();
    DoublePublisher outputPublisherLeft = NetworkTableInstance.getDefault().getDoubleTopic("LeftOutput").publish();
    StructPublisher<Rotation2d> gyroPublisher = NetworkTableInstance.getDefault().getStructTopic("CurrentRotation", Rotation2d.struct).publish();
    StructPublisher<Rotation2d> gyroPublisher2 = NetworkTableInstance.getDefault().getStructTopic("CurrentRotation2", Rotation2d.struct).publish();

    public MovementSubsystem() {
        leftMain.setCANTimeout(DriveConstants.CAN_TIMEOUT);
        rightMain.setCANTimeout(DriveConstants.CAN_TIMEOUT);
        leftFollow.setCANTimeout(DriveConstants.CAN_TIMEOUT);
        rightFollow.setCANTimeout(DriveConstants.CAN_TIMEOUT);

        SparkBaseConfig config = new SparkMaxConfig()
                .voltageCompensation(12)
                .smartCurrentLimit(DriveConstants.DRIVE_MOTOR_CURRENT_LIMIT);
        config.follow(leftMain);
        leftFollow.configure(config, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
        config.follow(rightMain);
        rightFollow.configure(config, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);

        // Remove following, then apply config to right leader
        config.disableFollowerMode();
        rightMain.configure(config, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
        // Set conifg to inverted and then apply to left leader. Set Left side inverted
        // so that postive values drive both sides forward
        config.inverted(true);
        leftMain.configure(config, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
        odometry = new DifferentialDriveOdometry(gyro.getRotation2d(), leftMain.getEncoder().getPosition(), rightMain.getEncoder().getPosition());
        odometry.resetPose(new Pose2d());
    }

    @Override
    public void periodic() {
        odometry.update(
                gyro.getRotation2d(),
                new DifferentialDriveWheelPositions(
                        leftMain.getAbsoluteEncoder().getPosition(),
                        rightMain.getAbsoluteEncoder().getPosition()
                )
        );
        posePublisher.accept(odometry.getPoseMeters());
        outputPublisherLeft.accept(leftMain.getOutputCurrent());
    }

    @Override
    public void simulationPeriodic() {
        odometry.update(
                new Rotation2d(gyro.getAngle()),
                new DifferentialDriveWheelPositions(
                        leftMainSim.getAbsoluteEncoderSim().getPosition(),
                        rightMainSim.getAbsoluteEncoderSim().getPosition()
                )
        );
        leftMainSim.iterate(
                Units.radiansPerSecondToRotationsPerMinute(drivetrainSim.getLeftVelocityMetersPerSecond()),
                12,
                0.02
        );
        rightMainSim.iterate(
                Units.radiansPerSecondToRotationsPerMinute(drivetrainSim.getRightVelocityMetersPerSecond()),
                12,
                0.02
        );
        drivetrainSim.setInputs(leftMainSim.getAppliedOutput(), rightMainSim.getAppliedOutput());
        drivetrainSim.update(0.02);
        gyroSim.setAngle(drivetrainSim.getHeading().getRadians());

        posePublisher.accept(odometry.getPoseMeters());
        gyroPublisher.accept(new Rotation2d(gyroSim.getAngle()));
        gyroPublisher2.accept(drivetrainSim.getHeading());
        outputPublisherLeft.accept(leftMain.getOutputCurrent());
        posePublisher2.accept(drivetrainSim.getPose());
    }

    // sets the speed of the drive motors
    public void driveArcade(double xSpeed, double zRotation) {
        drive.arcadeDrive(xSpeed, zRotation);
    }
}
