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
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.DifferentialDriveKinematics;
import edu.wpi.first.math.kinematics.DifferentialDriveOdometry;
import edu.wpi.first.math.kinematics.DifferentialDriveWheelPositions;
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

public class MovimientoSubsystem extends SubsystemBase {
  private final SparkMax MotorMovimientoIzquierdoLider;
  private final SparkMax MotorMovimientoIzquierdoSeguidor;
  private final SparkMax MotorMovimientoDerechoLider;
  private final SparkMax MotorMovimientoDerechoSeguidor;
  StructPublisher<Pose2d> posePublisher;

  public final RelativeEncoder leftEncoder;
  public final RelativeEncoder rightEncoder;
  private final DifferentialDriveOdometry odometry;
  private final Field2d field2d;

  private final DifferentialDrive drive;

  final double kDriveTickAMetros = (15.24 * Math.PI * 1.0 / 100.0) / 2.1;

  public MovimientoSubsystem() {
    // create brushed motors for drive
    MotorMovimientoIzquierdoLider = new SparkMax(DriveConstants.MotorMovimientoIzquierdoLider_ID, MotorType.kBrushed);
    MotorMovimientoIzquierdoSeguidor = new SparkMax(DriveConstants.MotorMovimientoIzquierdoSeguidor_ID,
        MotorType.kBrushed);
    MotorMovimientoDerechoLider = new SparkMax(DriveConstants.MotorMovimientoDerechoLider_ID, MotorType.kBrushed);
    MotorMovimientoDerechoSeguidor = new SparkMax(DriveConstants.MotorMovimientoDerechoSeguidor_ID, MotorType.kBrushed);

    // set up differential drive class
    drive = new DifferentialDrive(MotorMovimientoIzquierdoLider, MotorMovimientoDerechoLider);
    MotorMovimientoIzquierdoLider.setCANTimeout(DriveConstants.CAN_TIMEOUT);
    MotorMovimientoDerechoLider.setCANTimeout(DriveConstants.CAN_TIMEOUT);
    MotorMovimientoIzquierdoSeguidor.setCANTimeout(DriveConstants.CAN_TIMEOUT);
    MotorMovimientoDerechoSeguidor.setCANTimeout(DriveConstants.CAN_TIMEOUT);

    SparkMaxConfig config = new SparkMaxConfig();
    config.voltageCompensation(12);
    config.smartCurrentLimit(DriveConstants.DRIVE_MOTOR_CURRENT_LIMIT);

    config.follow(MotorMovimientoIzquierdoLider);
    MotorMovimientoIzquierdoSeguidor.configure(config, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
    config.follow(MotorMovimientoDerechoLider);
    MotorMovimientoDerechoSeguidor.configure(config, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);

    // Remove following, then apply config to right leader
    config.disableFollowerMode();
    MotorMovimientoDerechoLider.configure(config, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
    // Set conifg to inverted and then apply to left leader. Set Left side inverted
    // so that postive values drive both sides forward
    config.inverted(true);
    MotorMovimientoIzquierdoLider.configure(config, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);

    this.leftEncoder = MotorMovimientoIzquierdoLider.getEncoder();
    this.rightEncoder = MotorMovimientoDerechoLider.getEncoder();
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
            config, // The robot configuration
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

  // Set drive speeds using DifferentialDrive (tank drive)
  public void setDriveSpeed(double leftSpeed, double rightSpeed) {
    drive.tankDrive(leftSpeed, rightSpeed);
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
    return Kinematics.toChassisSpeeds(getLeftEncoderPosition(), getRightEncoderPosition(), Gyro.getInstance().getYawAngle2d());
  }
  

  // Arcade drive method (single joystick for forward/backward and rotation)
  public void driveArcade(double xSpeed, double zRotation) {
    drive.arcadeDrive(xSpeed, zRotation);
  }
}
