package frc.robot.subsystems;

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
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.DifferentialDriveKinematics;
import edu.wpi.first.math.kinematics.DifferentialDriveOdometry;
import edu.wpi.first.math.kinematics.Kinematics;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.networktables.StructPublisher;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.DriveConstants;
import frc.robot.util.Gyro;

public class MovementSubsystem extends SubsystemBase {
  private final SparkMax leftLeader;
  private final SparkMax leftFollow;
  private final SparkMax rightLeader;
  private final SparkMax rightFollow;

  StructPublisher<Pose2d> posePublisher;

  public final RelativeEncoder leftEncoder;
  public final RelativeEncoder rightEncoder;
  private final DifferentialDriveOdometry odometry;
  private final Field2d field2d;

  private final DifferentialDrive drive;

  final double kDriveTickAMetros = (15.24 * Math.PI * 1.0 / 100.0) / 2.1;

  public MovementSubsystem() {
    // Create brushed motors for drive
    leftLeader = new SparkMax(DriveConstants.LEFT_MOVEMENT_LEADER_MOTOR_ID, MotorType.kBrushed);
    leftFollow = new SparkMax(DriveConstants.LEFT_MOVEMENT_FOLLOW_MOTOR_ID, MotorType.kBrushed);
    rightLeader = new SparkMax(DriveConstants.RIGHT_MOVEMENT_LEADER_MOTOR_ID, MotorType.kBrushed);
    rightFollow = new SparkMax(DriveConstants.RIGHT_MOVEMENT_FOLLOW_MOTOR_ID, MotorType.kBrushed);

    // Set up differential drive class
    drive = new DifferentialDrive(leftLeader, rightLeader);
    leftLeader.setCANTimeout(DriveConstants.CAN_TIMEOUT);
    rightLeader.setCANTimeout(DriveConstants.CAN_TIMEOUT);
    leftFollow.setCANTimeout(DriveConstants.CAN_TIMEOUT);
    rightFollow.setCANTimeout(DriveConstants.CAN_TIMEOUT);

    configureMotors();
    configureOdometry();;
   
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
    this.leftEncoder = rightLeader.getEncoder();
    this.rightEncoder = leftLeader.getEncoder();
    odometry = new DifferentialDriveOdometry(Gyro.getInstance().getYawAngle2d(), getLeftEncoderPosition(), getRightEncoderPosition());
    DifferentialDriveKinematics diffdrivekinematics = new DifferentialDriveKinematics(0.546);

    field2d = new Field2d();
    SmartDashboard.putData(field2d);
    SmartDashboard.putData("Reset encoders", new InstantCommand(()->{
      leftEncoder.setPosition(0);
      rightEncoder.setPosition(0);
      odometry.resetPose(new Pose2d());
    }));
    posePublisher = NetworkTableInstance.getDefault().getStructTopic("Pose", Pose2d.struct).publish();
  }
  
  private void configureAutoBuilder() {
    RobotConfig configAuto;
    try{
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
              if (alliance.isPresent()) {
                return alliance.get() == DriverStation.Alliance.Red;
              }
              return false;
            },
            this // Reference to this subsystem to set requirements
    );
  }

  @Override
  public void periodic() {
    SmartDashboard.putData(drive);
    odometry.update(Gyro.getInstance().getYawAngle2d(), getLeftEncoderPosition(), getRightEncoderPosition());
    field2d.setRobotPose(odometry.getPoseMeters());
    SmartDashboard.putNumber("Left sensor position", getLeftEncoderPosition());
    SmartDashboard.putNumber("Right sensor position", getRightEncoderPosition());
    posePublisher.accept(odometry.getPoseMeters());
  }

  // Get encoder positions (converted to meters)
  public double getLeftEncoderPosition() {
    return leftEncoder.getPosition() * kDriveTickAMetros;
  }

  public double getRightEncoderPosition() {
    return -(rightEncoder.getPosition() * kDriveTickAMetros);
  }

  public Pose2d getPose(){
    return odometry.getPoseMeters();
  }

  public Pose2d resetPose(Pose2d pose2d){
    System.out.println(pose2d);
    odometry.resetPosition(Gyro.getInstance().getYawAngle2d(), getLeftEncoderPosition(), getRightEncoderPosition(), pose2d);
  }

  public ChassisSpeeds getRobotRelativeSpeeds(){
    double estimatedSpeed = (leftLeader.get() + rightLeader.get()) / 2.0;
    double estimatedRotation = Math.toRadians(Gyro.getInstance().getYawAngleVelocity());

    return new ChassisSpeeds(estimatedSpeed, 0, estimatedRotation);
  }

  

  // sets the speed of the drive motors
  public void driveArcade(double xSpeed, double zRotation) {
    drive.arcadeDrive(xSpeed, zRotation);
  }

  public void curvatureArcade(double xSpeed, double zRotation) {
    drive.curvatureDrive(xSpeed, zRotation, false);
  }
}
