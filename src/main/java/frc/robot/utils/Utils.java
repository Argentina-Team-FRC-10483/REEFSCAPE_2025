package frc.robot.utils;

public class Utils {
  /**
   * @param input    The value to apply the dead zone to
   * @param deadZone The dead zone limit
   * @return The value after applying the dead zone
   */
  public static double applyDeadZone(double input, double deadZone) {
    return Math.abs(input) < deadZone ? 0 : input;
  }
}