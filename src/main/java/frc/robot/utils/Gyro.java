package frc.robot.utils;

import com.studica.frc.AHRS;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class Gyro {
  private static Gyro instance = null;
  private final AHRS navx;

  private Gyro() {
    navx = new AHRS(AHRS.NavXComType.kMXP_SPI);
  }

  /**
   * Gets the current yaw angle.
   *
   * @return The angle in degrees limited to the range -180 to 180.
   */
  public double getYawAngle() {
    return -navx.getAngle();
  }

  /**
   * Gets the current roll angle.
   *
   * @return The angle in degrees.
   */
  public double getRollAngle() {
    return navx.getRoll();
  }

  /**
   * Gets the current pitch angle.
   *
   * @return The angle in degrees.
   */
  public double getPitchAngle() {
    return navx.getPitch();
  }

  /**
   * Gets the current rotation of the robot.
   *
   * @return The angle in degrees.
   */
  public double getRobotAngle() {
    return getYawAngle();
  }

  /**
   * Returns whether the NavX is currently connected and sending valid data
   *
   * @return whether the NavX is currently connected
   */
  public boolean navXConnected() {
    return navx.isConnected();
  }

  /**
   * Displays the angles on {@code SmartDashboard}.
   */
  public void outputValues() {
    SmartDashboard.putNumber("Yaw Angle", getYawAngle());
    SmartDashboard.putNumber("Roll Angle", getRollAngle());
    SmartDashboard.putNumber("Pitch Angle", getPitchAngle());

    SmartDashboard.putNumber("Robot Angle", getRobotAngle());
    SmartDashboard.putBoolean("Gyro Connected", navXConnected());
  }

  public static Gyro getInstance() {
    if (instance == null) {
      instance = new Gyro();
    }
    return instance;
  }

}