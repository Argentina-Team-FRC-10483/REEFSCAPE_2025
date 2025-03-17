package frc.robot.utils;

import com.studica.frc.AHRS;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.ADXRS450_Gyro;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class Gyro {
    private static Gyro instance = null;

    private final AHRS navx;
    private double resetRoll;
    private double resetPitch;
    private double resetYaw;

    private final ADXRS450_Gyro gyro;

    private Gyro() {
        navx = new AHRS(AHRS.NavXComType.kMXP_SPI);
        resetRoll = 0;
        resetPitch = 0;

        gyro = new ADXRS450_Gyro();
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
     * Gets the current yaw angle velocity in deg / s
     * 
     * @return The yaw angle rate in degrees.
     */
    public double getYawAngleVelocity() {
        return navx.getRate();
    }

    /**
     * Gets the current roll angle.
     * 
     * @return The angle in degrees.
     */
    public double getRollAngle() {
        return navx.getRoll() - resetRoll;
    }

    /**
     * Gets the current pitch angle.
     * 
     * @return The angle in degrees.
     */
    public double getPitchAngle() {
        return navx.getPitch() - resetPitch;
    }

    /**
     * Sets the current yaw angle to "0".
     */
    public void zeroYawAngle() {
//        navx.setAngleAdjustment(-getYawAngle());
//        navx.zeroYaw();
//        resetYaw = navx.getAngle();
    }

    /**
     * Sets the current roll angle to "0".
     */
    public void zeroRollAngle() {
        resetRoll = getRollAngle();
    }

    /**
     * Sets the current pitch angle to "0".
     */
    public void zeroPitchAngle() {
        resetPitch = getPitchAngle();
    }

    /**
     * Gets the current rotation of the robot.
     * 
     * @return The angle in degrees.
     */
    public double getRobotAngle() {
        return getYawAngle();
    }

    public void zeroAll() {
        zeroYawAngle();
        zeroPitchAngle();
        zeroRollAngle();
    }

    /**
     * Returns whether or not the NavX is currently connected and sending valid data
     * 
     * @return whether or not the NavX is currently connected
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
        // SmartDashboard.putNumber("Tilt Angle", getTiltAngle());
        SmartDashboard.putBoolean("Gyro Connected", navXConnected());
    }

    public static Gyro getInstance() {
        if (instance == null) {
            instance = new Gyro();
        }
        return instance;
    }

    public ADXRS450_Gyro getGyro() {
        return gyro;
    }
}