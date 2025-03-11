
package frc.robot.subsystems;

import java.util.List;

import com.revrobotics.sim.SparkRelativeEncoderSim;
import com.revrobotics.spark.SparkBase;
import com.revrobotics.spark.SparkClosedLoopController;
import edu.wpi.first.hal.simulation.SimDeviceDataJNI;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.estimator.DifferentialDrivePoseEstimator;
import edu.wpi.first.math.geometry.*;
import edu.wpi.first.math.kinematics.DifferentialDriveWheelSpeeds;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.units.measure.Voltage;
import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj.simulation.DifferentialDrivetrainSim;
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
import edu.wpi.first.hal.SimDouble;

import static edu.wpi.first.units.Units.Meters;
import static edu.wpi.first.units.Units.MetersPerSecond;
import static edu.wpi.first.units.Units.Volts;

public class MovementSubsystem extends SubsystemBase {
  final double kDriveTickAMetros = (15.24 * Math.PI * 1.0 / 100.0) / 2.1;
  private final SparkMax leftLeader = new SparkMax(DriveConstants.LEFT_MOVEMENT_LEADER_MOTOR_ID, MotorType.kBrushed);
  private final SparkMax leftFollow = new SparkMax(DriveConstants.LEFT_MOVEMENT_FOLLOW_MOTOR_ID, MotorType.kBrushed);
  private final SparkMax rightLeader = new SparkMax(DriveConstants.RIGHT_MOVEMENT_LEADER_MOTOR_ID, MotorType.kBrushed);
  private final SparkMax rightFollow = new SparkMax(DriveConstants.RIGHT_MOVEMENT_FOLLOW_MOTOR_ID, MotorType.kBrushed);
  private final SparkClosedLoopController leftController = leftLeader.getClosedLoopController();
  private final SparkClosedLoopController rightController = rightLeader.getClosedLoopController();
  private final DifferentialDrive drive;
  private final DifferentialDrivetrainSim driveSim;
  public RelativeEncoder leftEncoder;
  public RelativeEncoder rightEncoder;
  private SparkRelativeEncoderSim leftEncoderSim;
  private SparkRelativeEncoderSim rightEncoderSim;
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
  private final SimDouble gyroSimAngle;
  private final float kMaxVelocity = 10000.0f;
  private final float kMaxAcceleration = 10000.0f;
  private final TrapezoidProfile.Constraints m_constraints =
    new TrapezoidProfile.Constraints(kMaxVelocity, kMaxAcceleration);
  // Creates a SysIdRoutine
  SysIdRoutine routine = new SysIdRoutine(
    new SysIdRoutine.Config(),
    new SysIdRoutine.Mechanism(
      this::voltageDrive,
      this::logMotors,
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

  public MovementSubsystem() {
    // Set up differential drive class
//    drive = new DifferentialDrive(leftSpeed -> setLeftSpeed(leftSpeed), rightSpeed -> setRightSpeed(rightSpeed));
    drive = new DifferentialDrive(leftLeader, rightLeader);
    driveSim = new DifferentialDrivetrainSim(
      DCMotor.getCIM(2),
      7.29,                    // 7.29:1 gearing reduction.
      7.5,                     // MOI of 7.5 kg m^2 (from CAD model).
      60.0,                    // The mass of the robot is 60 kg.
      Units.inchesToMeters(3), // The robot uses 3" radius wheels.
      0.546,                  // The track width is 0.7112 meters.
      VecBuilder.fill(0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0)
    );
    leftLeader.setCANTimeout(DriveConstants.CAN_TIMEOUT);
    rightLeader.setCANTimeout(DriveConstants.CAN_TIMEOUT);
    leftFollow.setCANTimeout(DriveConstants.CAN_TIMEOUT);
    rightFollow.setCANTimeout(DriveConstants.CAN_TIMEOUT);

    configureMotors();
    configureOdometry();
    configureAutoBuilder();
    // Unidades son metros.
    cameraInterFace = new CameraInterFace(
      List.of(
        new PositionCamera(
          new PhotonCamera("Camera_1"),
          new Transform3d(new Translation3d(0.5, 0.0, 0.5), new Rotation3d(0, 0, 0))
        ),
        new PositionCamera(
          new PhotonCamera("Camera_2"),
          new Transform3d(new Translation3d(0.275, 0.325, 0.42), new Rotation3d(0, 0, 3.14))
        ),
        new PositionCamera(
          new PhotonCamera("Camera_3"),
          new Transform3d(new Translation3d(0.5, 0.0, 0.5), new Rotation3d(0, 0, 0))
        )
      ),
      odometry::addVisionMeasurement
    );
    int dev = SimDeviceDataJNI.getSimDeviceHandle("navX-Sensor[4]");
    gyroSimAngle = new SimDouble(SimDeviceDataJNI.getSimValueHandle(dev, "Yaw"));
  }

  private void configureMotors() {
    SparkMaxConfig config = new SparkMaxConfig();
    config.voltageCompensation(12);
    config.smartCurrentLimit(DriveConstants.DRIVE_MOTOR_CURRENT_LIMIT);
    config.closedLoop.
      p(0.3)
      .i(0)
      .d(0);
    config.closedLoop.maxMotion
      .maxAcceleration(0.01)
      .maxVelocity(0.01);
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
    this.rightEncoderSim = new SparkRelativeEncoderSim(rightLeader);
    this.leftEncoderSim = new SparkRelativeEncoderSim(leftLeader);
    odometry = new DifferentialDrivePoseEstimator(
      kinematics,
      Gyro.getInstance().getYawAngle2d(),
      getLeftEncoderPosition(),
      getRightEncoderPosition(),
      new Pose2d()
//      VecBuilder.fill(0.05, 0.05, Units.degreesToRadians(5)),
//      VecBuilder.fill(0.5, 0.5, Units.degreesToRadians(30))
    );

    field2d = new Field2d();
    SmartDashboard.putData(field2d);
    SmartDashboard.putData("Reset encoders", new InstantCommand(this::resetOdometry));
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
    PPLTVController controller = new PPLTVController(0.02);
//    controller.setEnabled(false);
    AutoBuilder.configure(
      this::getPose, // Robot pose supplier
      this::resetPose, // Method to reset odometry (will be called if your auto has a starting pose)
      this::getRobotRelativeSpeeds, // ChassisSpeeds supplier. MUST BE ROBOT RELATIVE
      (speeds, feedforwards) -> driveRobotRelative(speeds), // Method that will drive the robot given ROBOT RELATIVE ChassisSpeeds. Also optionally outputs individual module feedforwards
      controller, // PPLTVController is the built in path following controller for differential drive trains
      configAuto, // The robot configuration
      () -> {
        // Boolean supplier that controls when the path will be mirrored for the red alliance
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
    odometry.update(Rotation2d.fromDegrees(Gyro.getInstance().getYawAngle()), getLeftEncoderPosition(), getRightEncoderPosition());
    field2d.setRobotPose(getPose());
    SmartDashboard.putNumber("Left sensor position", getLeftEncoderPosition());
    SmartDashboard.putNumber("Right sensor position", getRightEncoderPosition());

    leftVelocity = rpmToMetersPerSecond(leftEncoder.getVelocity());
    rightVelocity = rpmToMetersPerSecond(rightEncoder.getVelocity());

    SmartDashboard.putNumber("Left Motor Velocity", leftVelocity);
    SmartDashboard.putNumber("Right Motor Velocity", rightVelocity);
    posePublisher.accept(getPose());
    cameraInterFace.periodic();
    Gyro.getInstance().outputValues();
  }

  @Override
  public void simulationPeriodic() {
    driveSim.setInputs(
      leftLeader.get() * RobotController.getInputVoltage(),
      rightLeader.get() * RobotController.getInputVoltage()
    );
    driveSim.update(0.02);
    leftEncoderSim.setPosition(driveSim.getLeftPositionMeters() / kDriveTickAMetros);
    leftEncoderSim.setVelocity(driveSim.getLeftVelocityMetersPerSecond() / kDriveTickAMetros);
    rightEncoderSim.setPosition(driveSim.getRightPositionMeters() / kDriveTickAMetros);
    rightEncoderSim.setVelocity(driveSim.getRightVelocityMetersPerSecond() / kDriveTickAMetros);
    gyroSimAngle.set(driveSim.getHeading().getDegrees());
    SmartDashboard.putNumber("Simulated Heading", driveSim.getHeading().getDegrees());
    posePublisher.accept(driveSim.getPose());
  }

  // Get encoder positions (converted to meters)
  public double getLeftEncoderPosition() {
    return leftEncoder.getPosition() * kDriveTickAMetros;
  }

  public double getRightEncoderPosition() {
    return -(rightEncoder.getPosition() * kDriveTickAMetros);
  }

  public Pose2d getPose() {
    // If is simulated, return drivetrainSim pose
    return RobotBase.isReal() ? odometry.getEstimatedPosition() : driveSim.getPose();
  }

  public void resetPose(Pose2d pose2d) {
    resetOdometry();
    driveSim.setPose(pose2d);
    odometry.resetPosition(Gyro.getInstance().getYawAngle2d(), getLeftEncoderPosition(), getRightEncoderPosition(), pose2d);
  }

  public void setSpeeds(double leftSpeed, double rightSpeed) {
    setLeftSpeed(leftSpeed);
    setRightSpeed(rightSpeed);
  }

  public void setLeftSpeed(double leftSpeed) {
    SmartDashboard.putNumber("Left Target", leftSpeed);
    SmartDashboard.putNumber("Left current", leftEncoder.getVelocity());
    leftController.setReference(leftSpeed, SparkBase.ControlType.kVelocity);
  }

  public void setRightSpeed(double rightSpeed) {
    SmartDashboard.putNumber("Right Target", rightSpeed);
    SmartDashboard.putNumber("Right current", rightEncoder.getVelocity());
    rightController.setReference(rightSpeed, SparkBase.ControlType.kVelocity);
  }

  public ChassisSpeeds getRobotRelativeSpeeds() {
    ChassisSpeeds chassisSpeeds = kinematics.toChassisSpeeds(new DifferentialDriveWheelSpeeds(
      getLeftVelocity(),
      getRightVelocity()
    ));
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
      false
    );
  }

  private double getRightVelocity() {
    return rightEncoder.getVelocity() * kDriveTickAMetros;
  }

  private double getLeftVelocity() {
    return leftEncoder.getVelocity() * kDriveTickAMetros;
  }

  public static double rpmToMetersPerSecond(double rpm) {
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

  private void voltageDrive(Voltage voltage) {
    leftLeader.setVoltage(voltage);
    rightLeader.setVoltage(voltage);
  }

  private void logMotors(SysIdRoutineLog log) {
    recordMotor(log, "drive-left", leftLeader.getAppliedOutput(), getLeftEncoderPosition(), leftVelocity);
    recordMotor(log, "drive-right", rightLeader.getAppliedOutput(), getRightEncoderPosition(), rightVelocity);
  }

  private void resetOdometry() {
    leftEncoder.setPosition(0);
    rightEncoder.setPosition(0);
    leftEncoderSim.setPosition(0);
    rightEncoderSim.setPosition(0);
    driveSim.setPose(new Pose2d());
    odometry.resetPose(new Pose2d());
  }
}