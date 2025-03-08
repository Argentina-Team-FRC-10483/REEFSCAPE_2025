
package frc.robot.subsystems;

import java.util.List;

import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.estimator.DifferentialDrivePoseEstimator;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.sysid.SysIdRoutineLog;
import org.photonvision.PhotonCamera;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.config.RobotConfig;
import com.pathplanner.lib.controllers.PPLTVController;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.SparkMaxConfig;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.DifferentialDriveKinematics;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.networktables.StructPublisher;
import edu.wpi.first.units.measure.MutDistance;
import edu.wpi.first.units.measure.MutLinearVelocity;
import edu.wpi.first.units.measure.MutVoltage;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import frc.robot.Constants.DriveConstants;
import frc.robot.utils.CameraInterFace;
import frc.robot.utils.Gyro;
import frc.robot.utils.PositionCamera;

import static edu.wpi.first.units.Units.Meters;
import static edu.wpi.first.units.Units.MetersPerSecond;
import static edu.wpi.first.units.Units.Volts;

public class MovementSubsystem extends SubsystemBase {
  final double kDriveTickAMetros = (15.24 * Math.PI * 1.0 / 100.0) / 2.1;
  private final SparkMax leftLeader = new SparkMax(DriveConstants.LEFT_MOVEMENT_LEADER_MOTOR_ID, MotorType.kBrushed);
  private final SparkMax leftFollow = new SparkMax(DriveConstants.LEFT_MOVEMENT_FOLLOW_MOTOR_ID, MotorType.kBrushed);
  private final SparkMax rightLeader = new SparkMax(DriveConstants.RIGHT_MOVEMENT_LEADER_MOTOR_ID, MotorType.kBrushed);
  private final SparkMax rightFollow = new SparkMax(DriveConstants.RIGHT_MOVEMENT_FOLLOW_MOTOR_ID, MotorType.kBrushed);
  private final DifferentialDrive drive;
  public RelativeEncoder leftEncoder;
  public RelativeEncoder rightEncoder;
  double leftVelocity;
  double rightVelocity;
  private final CameraInterFace cameraInterFace;
  final DifferentialDriveKinematics kinematics = new DifferentialDriveKinematics(0.546);
  private DifferentialDrivePoseEstimator odometry;
  private Field2d field2d;
  StructPublisher<Pose2d> posePublisher;
  private final MutVoltage m_appliedVoltage = Volts.mutable(0);
  private final MutDistance m_distance = Meters.mutable(0);  // Mutable holder for unit-safe linear distance values, persisted to avoid reallocation.
  private final MutLinearVelocity m_velocity = MetersPerSecond.mutable(0);  // Mutable holder for unit-safe linear velocity values, persisted to avoid reallocation.

  // Creates a SysIdRoutine
  SysIdRoutine routine = new SysIdRoutine(
    new SysIdRoutine.Config(),
    new SysIdRoutine.Mechanism(
      voltage -> {
        leftLeader.setVoltage(voltage);
        rightLeader.setVoltage(voltage);
      }, log -> {
      recordMotor(log, "drive-left", getAppliedOutput(), getLeftEncoderPosition(), leftVelocity);
      recordMotor(log, "drive-right", rightLeader.getAppliedOutput(), getRightEncoderPosition(), rightVelocity);
    },
      this
    )
  );

  private void recordMotor(SysIdRoutineLog log, String name, double appliedOutput, double encoderPosition, double velocity) {
    // Record a frame for a pair of motors.  Since these share an encoder, we consider the entire group to be one motor.
    log.motor(name)
      .voltage(m_appliedVoltage.mut_replace(appliedOutput * RobotController.getBatteryVoltage(), Volts))
      .linearPosition(m_distance.mut_replace(encoderPosition, Meters))
      .linearVelocity(m_velocity.mut_replace(velocity, MetersPerSecond));
  }

  private double getAppliedOutput() {
    return leftLeader.getAppliedOutput();
  }

  public MovementSubsystem() {
    // Set up differential drive class
    drive = new DifferentialDrive(leftLeader, rightLeader);
    leftLeader.setCANTimeout(DriveConstants.CAN_TIMEOUT);
    rightLeader.setCANTimeout(DriveConstants.CAN_TIMEOUT);
    leftFollow.setCANTimeout(DriveConstants.CAN_TIMEOUT);
    rightFollow.setCANTimeout(DriveConstants.CAN_TIMEOUT);

    configureMotors();
    configureOdometry();
    configureAutoBuilder();
    // TODO: Investigar las unidades del Trasfrom3d, creemos metros.
    cameraInterFace = new CameraInterFace(
      List.of(
        new PositionCamera(
          new PhotonCamera("Camera_1"),
          new Transform3d(new Translation3d(0.5, 0.0, 0.5), new Rotation3d(0, 0, 0))
        ),
        new PositionCamera(
          new PhotonCamera("Camera_2"),
          new Transform3d(new Translation3d(0.5, 0.0, 0.5), new Rotation3d(0, 0, 0))
        ),
        new PositionCamera(
          new PhotonCamera("Camera_3"),
          new Transform3d(new Translation3d(0.5, 0.0, 0.5), new Rotation3d(0, 0, 0))
        )
      ),
      odometry::addVisionMeasurement
    );
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
    this.rightEncoder = rightLeader.getEncoder();
    this.leftEncoder = leftLeader.getEncoder();
    odometry = new DifferentialDrivePoseEstimator(
      kinematics,
      Gyro.getInstance().getYawAngle2d(),
      getLeftEncoderPosition(),
      getRightEncoderPosition(),
      new Pose2d(),
      VecBuilder.fill(0.05, 0.05, Units.degreesToRadians(5)),
      VecBuilder.fill(0.5, 0.5, Units.degreesToRadians(30))
    );

    field2d = new Field2d();
    SmartDashboard.putData(field2d);
    SmartDashboard.putData("Reset encoders", new InstantCommand(() -> {
      leftEncoder.setPosition(0);
      rightEncoder.setPosition(0);
      odometry.resetPose(new Pose2d());
    }));
    posePublisher = NetworkTableInstance.getDefault().getStructTopic("Pose", Pose2d.struct).publish();
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
    AutoBuilder.configure(
      this::getPose, // Robot pose supplier
      this::resetPose, // Method to reset odometry (will be called if your auto has a starting pose)
      this::getRobotRelativeSpeeds, // ChassisSpeeds supplier. MUST BE ROBOT RELATIVE
      (speeds, feedforwards) -> driveRobotRelative(speeds), // Method that will drive the robot given ROBOT RELATIVE ChassisSpeeds. Also optionally outputs individual module feedforwards
      new PPLTVController(0.02), // PPLTVController is the built in path following controller for differential drive trains
      configAuto, // The robot configuration
      () -> {
        // Boolean supplier that controls when the path will be mirrored for the red alliance
        // This will flip the path being followed to the red side of the field.
        // THE ORIGIN WILL REMAIN ON THE BLUE SIDE

        var alliance = DriverStation.getAlliance();
        return alliance.filter(value -> value == DriverStation.Alliance.Red).isPresent();
      },
      this // Reference to this subsystem to set requirements
    );
  }

  @Override
  public void periodic() {
    SmartDashboard.putData(drive);
    odometry.update(Gyro.getInstance().getYawAngle2d(), getLeftEncoderPosition(), getRightEncoderPosition());
    field2d.setRobotPose(odometry.getEstimatedPosition());
    SmartDashboard.putNumber("Left sensor position", getLeftEncoderPosition());
    SmartDashboard.putNumber("Right sensor position", getRightEncoderPosition());

    leftVelocity = calculoRPM(leftEncoder.getVelocity());
    rightVelocity = calculoRPM(rightEncoder.getVelocity());

    SmartDashboard.putNumber("Left Motor Velocity", leftVelocity);
    SmartDashboard.putNumber("Right Motor Velocity", rightVelocity);
    posePublisher.accept(odometry.getEstimatedPosition());
    cameraInterFace.periodic();
  }

  // Get encoder positions (converted to meters)
  public double getLeftEncoderPosition() {
    return leftEncoder.getPosition() * kDriveTickAMetros;
  }

  public double getRightEncoderPosition() {
    return -(rightEncoder.getPosition() * kDriveTickAMetros);
  }

  public Pose2d getPose() {
    System.out.println("getPose");
    return odometry.getEstimatedPosition();
    //return new Pose2d(odometry.getPoseMeters().getX()/10.0, odometry.getPoseMeters().getY()/10.0, odometry.getPoseMeters().getRotation());
  }

  public Pose2d resetPose(Pose2d pose2d) {
    System.out.println(pose2d);
    odometry.resetPosition(Gyro.getInstance().getYawAngle2d(), getLeftEncoderPosition(), getRightEncoderPosition(), pose2d);
    System.out.println("resetPose");
    return odometry.getEstimatedPosition();
  }

  public ChassisSpeeds getRobotRelativeSpeeds() {
    double vx = (leftVelocity + rightVelocity) / 2;
    double vy = 0.0;
    SmartDashboard.putNumber("vx", vx);
    SmartDashboard.putNumber("leftVelocity", leftVelocity);
    SmartDashboard.putNumber("rightVelocity", rightVelocity);
    SmartDashboard.putNumber("robotanglevelocity", Gyro.getInstance().getRobotAngleVelocity());
    return new ChassisSpeeds(vx, vy, Gyro.getInstance().getRobotAngleVelocity());
  }

  public void driveRobotRelative(ChassisSpeeds speeds) {
    var wheelSpeeds = kinematics.toWheelSpeeds(speeds);
    drive.tankDrive(wheelSpeeds.leftMetersPerSecond * 0.11, wheelSpeeds.rightMetersPerSecond * 0.11);
  }

  public static double calculoRPM(double rpm) {
    return (2 * Math.PI * 0.0762 * rpm) / 60;
  }

  // sets the speed of the drive motors
  public void driveArcade(double xSpeed, double zRotation) {
    drive.arcadeDrive(xSpeed, zRotation, false);
  }

  public void curvatureArcade(double xSpeed, double zRotation) {
    drive.curvatureDrive(xSpeed, zRotation, false); //hay que probar
  }

  // Commands for testing
  public Command sysIdQuasistatic(SysIdRoutine.Direction direction) {
    return routine.quasistatic(direction);
  }

  public Command sysIdDynamic(SysIdRoutine.Direction direction) {
    return routine.dynamic(direction);
  }
}