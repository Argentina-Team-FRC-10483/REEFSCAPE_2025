
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

import edu.wpi.first.math.estimator.DifferentialDrivePoseEstimator;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.DifferentialDriveKinematics;
import edu.wpi.first.math.kinematics.DifferentialDriveWheelSpeeds;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.networktables.StructPublisher;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.DriveConstants;

import frc.robot.utils.Gyro;

public class MovementSubsystem extends SubsystemBase {
  private final SparkMax leftLeader;
  private final SparkMax leftFollow;
  private final SparkMax rightLeader;
  private final SparkMax rightFollow;
  private final RelativeEncoder leftEncoder;
  private final RelativeEncoder rightEncoder;
  private final DifferentialDrive drive;
  final DifferentialDriveKinematics kinematics = new DifferentialDriveKinematics(0.546);
  private DifferentialDrivePoseEstimator odometry;
  private Field2d field2d;
  StructPublisher<Pose2d> posePublisher;

  static final double kDriveRotToMts = (15.24 * Math.PI * 1.0 / 100.0);
  static final double kDriveRPMToMps = (2 * Math.PI * 0.0762) / 60;

  public MovementSubsystem() {
    // Create brushed motors for drive
    leftLeader = new SparkMax(DriveConstants.LEFT_LEADER_CAN_ID, MotorType.kBrushed);
    leftFollow = new SparkMax(DriveConstants.LEFT_FOLLOW_CAN_ID, MotorType.kBrushed);
    rightLeader = new SparkMax(DriveConstants.RIGHT_LEADER_CAN_ID, MotorType.kBrushed);
    rightFollow = new SparkMax(DriveConstants.RIGHT_FOLLOW_CAN_ID, MotorType.kBrushed);

    // Set up differential drive class
    drive = new DifferentialDrive(leftLeader, rightLeader);
    leftLeader.setCANTimeout(DriveConstants.CAN_TIMEOUT);
    rightLeader.setCANTimeout(DriveConstants.CAN_TIMEOUT);
    leftFollow.setCANTimeout(DriveConstants.CAN_TIMEOUT);
    rightFollow.setCANTimeout(DriveConstants.CAN_TIMEOUT);

    leftEncoder = leftLeader.getEncoder();
    rightEncoder = rightLeader.getEncoder();

    configureMotors();
    configureAutoBuilder();
    configureOdometry();
  }

  private void configureMotors() {
    SparkMaxConfig config = new SparkMaxConfig();
    config.voltageCompensation(12);
    config.smartCurrentLimit(DriveConstants.DRIVE_MOTOR_CURRENT_LIMIT);

    config.follow(leftLeader);
    leftFollow.configure(config, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
    config.follow(rightLeader);
    rightFollow.configure(config, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);

    // Remove following, then apply config to right leader
    config.disableFollowerMode();
    rightLeader.configure(config, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
    // Set conifg to inverted and then apply to left leader.
    // Set Left side inverted so that positive values drive both sides forward
    config.inverted(true);
    leftLeader.configure(config, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
  }

  private void configureOdometry() {
    odometry = new DifferentialDrivePoseEstimator(
      kinematics,
      Rotation2d.fromDegrees(Gyro.getInstance().getYawAngle()),
      getLeftEncoderPosition(),
      getRightEncoderPosition(),
      new Pose2d());
    field2d = new Field2d();
    SmartDashboard.putData(field2d);
    SmartDashboard.putData("Reset encoders", new InstantCommand(this::resetOdometry));
    posePublisher = NetworkTableInstance.getDefault().getStructTopic("Pose", Pose2d.struct).publish();
    resetOdometry();
  }

  private void configureAutoBuilder() {
    RobotConfig configAuto = null;
    try {
      configAuto = RobotConfig.fromGUISettings();
    } catch (Exception e) {
      // Handle exception as needed
      e.printStackTrace();
    }

    // Configure AutoBuilder last
    PPLTVController controller = new PPLTVController(0.02);
    // controller.setEnabled(false);
    AutoBuilder.configure(
      this::getPose, // Robot pose supplier
      this::resetPose, // Method to reset odometry (will be called if your auto has a starting pose)
      this::getRobotRelativeSpeeds, // ChassisSpeeds supplier. MUST BE ROBOT RELATIVE
      (speeds, feedforwards) -> driveRobotRelative(speeds), // Method that will drive the robot given ROBOT RELATIVE
      // ChassisSpeeds. Also optionally outputs individual
      // module feedforwards
      controller, // PPLTVController is the built in path following controller for differential
      // drive trains
      configAuto, // The robot configuration
      () -> {
        // Boolean supplier that controls when the path will be mirrored for the red
        // alliance
        // This will flip the path being followed to the red side of the field.
        // THE ORIGIN WILL REMAIN ON THE BLUE SIDE

        var alliance = DriverStation.getAlliance();
        return alliance.filter(value -> value == DriverStation.Alliance.Blue).isPresent();
      },
      this // Reference to this subsystem to set requirements
    );
  }

  @Override
  public void periodic() {
    SmartDashboard.putData(drive);
    odometry.update(
      Rotation2d.fromDegrees(Gyro.getInstance().getYawAngle()),
      getLeftEncoderPosition(),
      getRightEncoderPosition()
    );
    field2d.setRobotPose(getPose());
    posePublisher.accept(getPose());
    // cameraInterFace.periodic();
    Gyro.getInstance().outputValues();
  }

  // Get encoder positions (converted to meters)
  public double getLeftEncoderPosition() {
    return leftEncoder.getPosition() * kDriveRotToMts;
  }

  public double getRightEncoderPosition() {
    return -rightEncoder.getPosition() * kDriveRotToMts;
  }

  public Pose2d getPose() {
    // If is simulated, return drivetrainSim pose
    return odometry.getEstimatedPosition();
  }

  public void resetPose(Pose2d pose2d) {
    odometry.resetPose(pose2d);
  }

  public ChassisSpeeds getRobotRelativeSpeeds() {
    ChassisSpeeds chassisSpeeds = kinematics.toChassisSpeeds(
      new DifferentialDriveWheelSpeeds(
        leftEncoder.getVelocity() * kDriveRPMToMps,
        -rightEncoder.getVelocity() * kDriveRPMToMps
      )
    );
    SmartDashboard.putNumber("VX m/s", chassisSpeeds.vxMetersPerSecond);
    SmartDashboard.putNumber("VY m/s", chassisSpeeds.vyMetersPerSecond);
    SmartDashboard.putNumber("VO r/s", chassisSpeeds.omegaRadiansPerSecond);
    return chassisSpeeds;
  }

  public void driveRobotRelative(ChassisSpeeds speeds) {
    var wheelSpeeds = kinematics.toWheelSpeeds(speeds);
    drive.tankDrive(
      wheelSpeeds.leftMetersPerSecond,
      wheelSpeeds.rightMetersPerSecond,
      false);
  }

  // sets the speed of the drive motors
  public void driveArcade(double xSpeed, double zRotation) {
    drive.arcadeDrive(xSpeed, zRotation, false);
  }

  public void curvatureArcade(double xSpeed, double zRotation) {
    drive.curvatureDrive(xSpeed, zRotation, false);
  }

  private void resetOdometry() {
    Gyro.getInstance().zeroYawAngle();
    leftEncoder.setPosition(0);
    rightEncoder.setPosition(0);
    odometry.resetPose(new Pose2d(0, 0, new Rotation2d(0)));
  }
}