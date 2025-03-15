
package frc.robot.subsystems;

import java.util.List;

import edu.wpi.first.hal.simulation.SimDeviceDataJNI;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.estimator.DifferentialDrivePoseEstimator;
import edu.wpi.first.math.geometry.*;
import edu.wpi.first.math.kinematics.DifferentialDriveWheelSpeeds;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.simulation.DifferentialDrivetrainSim;
import frc.robot.utils.MovementMotor;
import org.photonvision.PhotonCamera;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.config.RobotConfig;
import com.pathplanner.lib.controllers.PPLTVController;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.DifferentialDriveKinematics;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.networktables.StructPublisher;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.utils.CameraInterFace;
import frc.robot.utils.Gyro;
import frc.robot.utils.PositionCamera;
import edu.wpi.first.hal.SimDouble;

public class MovementSubsystem extends SubsystemBase {
  private final MovementMotor left;
  private final MovementMotor right;
  private final DifferentialDrive drive;
  private final DifferentialDrivetrainSim driveSim;
  private final CameraInterFace cameraInterFace;
  final DifferentialDriveKinematics kinematics = new DifferentialDriveKinematics(0.546);
  private DifferentialDrivePoseEstimator odometry;
  private Field2d field2d;
  StructPublisher<Pose2d> posePublisher;
  private final SimDouble gyroSimAngle;

  public MovementSubsystem(MovementMotor left, MovementMotor right) {
    this.left = left;
    this.right = right;
    drive = new DifferentialDrive(left::set, right::set);
    driveSim = new DifferentialDrivetrainSim(
      DCMotor.getCIM(2),
      7.29,                    // 7.29:1 gearing reduction.
      7.5,                     // MOI of 7.5 kg m^2 (from CAD model).
      60.0,                    // The mass of the robot is 60 kg.
      Units.inchesToMeters(3), // The robot uses 3" radius wheels.
      0.546,                  // The track width is 0.7112 meters.
      VecBuilder.fill(0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0)
    );
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

  private void configureOdometry() {
    odometry = new DifferentialDrivePoseEstimator(
      kinematics,
      Gyro.getInstance().getYawAngle2d(),
      getLeftEncoderPosition(),
      getRightEncoderPosition(),
      new Pose2d()
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
    odometry.update(
      Rotation2d.fromDegrees(Gyro.getInstance().getYawAngle()),
      getLeftEncoderPosition(),
      getRightEncoderPosition()
    );
    field2d.setRobotPose(getPose());
    posePublisher.accept(getPose());
    cameraInterFace.periodic();
    left.periodic();
    right.periodic();
    Gyro.getInstance().outputValues();
  }

  @Override
  public void simulationPeriodic() {
    driveSim.setInputs(
      left.leader.getAppliedOutput() * RobotController.getBatteryVoltage(),
      right.leader.getAppliedOutput() * RobotController.getBatteryVoltage()
    );
    driveSim.update(0.02);
    left.updateSimulation(driveSim.getLeftPositionMeters(), driveSim.getLeftVelocityMetersPerSecond());
    right.updateSimulation(driveSim.getRightPositionMeters(), driveSim.getRightVelocityMetersPerSecond());
    gyroSimAngle.set(driveSim.getHeading().getDegrees());
    SmartDashboard.putNumber("Simulated Heading", driveSim.getHeading().getDegrees());
    posePublisher.accept(driveSim.getPose());
  }

  // Get encoder positions (converted to meters)
  public double getLeftEncoderPosition() {
    return left.encoder.getPosition();
  }

  public double getRightEncoderPosition() {
    return -right.encoder.getPosition();
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

  public ChassisSpeeds getRobotRelativeSpeeds() {
    ChassisSpeeds chassisSpeeds = kinematics.toChassisSpeeds(new DifferentialDriveWheelSpeeds(
      left.encoder.getVelocity(),
      right.encoder.getVelocity()
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

  // sets the speed of the drive motors
  public void driveArcade(double xSpeed, double zRotation) {
    drive.arcadeDrive(xSpeed, zRotation, false);
  }

  private void resetOdometry() {
    driveSim.setPose(new Pose2d());
    odometry.resetPose(new Pose2d());
  }
}