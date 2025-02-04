package frc.robot.subsystems;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.config.RobotConfig;
import com.pathplanner.lib.controllers.PPLTVController;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.SparkMaxConfig;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.DriveConstants;

public class MovimientoSubsystem extends SubsystemBase {
  private final SparkMax MotorMovimientoIzquierdoLider;
  private final SparkMax MotorMovimientoIzquierdoSeguidor;
  private final SparkMax MotorMovimientoDerechoLider;
  private final SparkMax MotorMovimientoDerechoSeguidor;

  private final DifferentialDrive drive;

  private Pose2d currentPose = new Pose2d();

  // AGREGAR GYRO DE NAVX

  public MovimientoSubsystem() {
    // Crear motores brushed para la conducción
    MotorMovimientoIzquierdoLider = new SparkMax(DriveConstants.MotorMovimientoIzquierdoLider_ID, MotorType.kBrushed);
    MotorMovimientoIzquierdoSeguidor = new SparkMax(DriveConstants.MotorMovimientoIzquierdoSeguidor_ID,
        MotorType.kBrushed);
    MotorMovimientoDerechoLider = new SparkMax(DriveConstants.MotorMovimientoDerechoLider_ID, MotorType.kBrushed);
    MotorMovimientoDerechoSeguidor = new SparkMax(DriveConstants.MotorMovimientoDerechoSeguidor_ID, MotorType.kBrushed);

    // Configurar la clase DifferentialDrive
    drive = new DifferentialDrive(MotorMovimientoIzquierdoLider, MotorMovimientoDerechoLider);
    MotorMovimientoIzquierdoLider.setCANTimeout(DriveConstants.CAN_TIMEOUT);
    MotorMovimientoDerechoLider.setCANTimeout(DriveConstants.CAN_TIMEOUT);
    MotorMovimientoIzquierdoSeguidor.setCANTimeout(DriveConstants.CAN_TIMEOUT);
    MotorMovimientoDerechoSeguidor.setCANTimeout(DriveConstants.CAN_TIMEOUT);

    SparkMaxConfig motorConfig = new SparkMaxConfig();
    motorConfig.voltageCompensation(12);
    motorConfig.smartCurrentLimit(DriveConstants.DRIVE_MOTOR_CURRENT_LIMIT);

    motorConfig.follow(MotorMovimientoIzquierdoLider);
    MotorMovimientoIzquierdoSeguidor.configure(motorConfig, ResetMode.kResetSafeParameters,
        PersistMode.kPersistParameters);
    motorConfig.follow(MotorMovimientoDerechoLider);
    MotorMovimientoDerechoSeguidor.configure(motorConfig, ResetMode.kResetSafeParameters,
        PersistMode.kPersistParameters);

    // Eliminar modo seguidor y aplicar configuración al líder derecho
    motorConfig.disableFollowerMode();
    MotorMovimientoDerechoLider.configure(motorConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);

    // Configurar el líder izquierdo como invertido para avanzar correctamente
    motorConfig.inverted(true);
    MotorMovimientoIzquierdoLider.configure(motorConfig, ResetMode.kResetSafeParameters,
        PersistMode.kPersistParameters);

    // RESETEAR GYRO NAVX

    // Cargar la configuración del robot desde la GUI
    try {
      RobotConfig config = RobotConfig.fromGUISettings();
      // Configurar AutoBuilder
      AutoBuilder.configure(
          this::getPose, // Proveedor de la posición del robot
          this::resetPose, // Método para reiniciar la odometría
          this::getRobotRelativeSpeeds, // Proveedor de ChassisSpeeds en relación al robot
          (speeds, feedforwards) -> driveRobotRelative(speeds), // Método para conducir en base a ChassisSpeeds
          new PPLTVController(0.02), // Controlador de trayectoria
          config, // Configuración del robot
          () -> {
            var alliance = DriverStation.getAlliance();
            if (alliance.isPresent()) {
              return alliance.get() == DriverStation.Alliance.Red;
            }
            return false;
          },
          this // Referencia a este subsistema para establecer requisitos
      );
    } catch (Exception e) {
      DriverStation.reportError("Failed to load PathPlanner config and configure AutoBuilder", e.getStackTrace());
    }
  }

  // config = new RobotConfig(74.088, 6.883, null, 0.546);
  @Override
  public void periodic() {
    SmartDashboard.putData(drive);
  }

  // Método para controlar el robot en modo arcade
  public void driveArcade(double xSpeed, double zRotation) {
    drive.arcadeDrive(xSpeed, zRotation);
  }

  public Pose2d getPose() {
    return currentPose;
  }

  public void resetPose(Pose2d pose) {
    currentPose = pose;
    // RESET GYRO NAVX
  }

  public ChassisSpeeds getRobotRelativeSpeeds() {
    // Debes implementar esto según cómo obtienes las velocidades
    return new ChassisSpeeds(0, 0, 0);
  }

  public void driveRobotRelative(ChassisSpeeds speeds) {
    driveArcade(speeds.vxMetersPerSecond, speeds.omegaRadiansPerSecond);
  }
}
