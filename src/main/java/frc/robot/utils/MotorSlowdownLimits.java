package frc.robot.utils;

import frc.robot.Constants;

import java.util.function.Consumer;
import java.util.function.Supplier;

/**
 * Limits the speed of a motor based on its position.
 *
 * <p>When the motor is close to the lower bound, the speed is limited to a fraction of the original
 * speed. The same happens when the motor is close to the upper bound.
 */
public class MotorSlowdownLimits {
  private final double lowerBound;
  private final double upperBound;
  private final double slowdownRange;
  private final Supplier<Double> getPosition;
  private final Consumer<Double> setSpeed;

  public MotorSlowdownLimits(double lowerBound, double upperBound, double slowdownRange, Supplier<Double> getPosition, Consumer<Double> setSpeed) {
    this.lowerBound = lowerBound;
    this.upperBound = upperBound;
    this.slowdownRange = slowdownRange;
    this.getPosition = getPosition;
    this.setSpeed = setSpeed;
  }

  public void move(double speed) {
    double currentPosition = getPosition.get();
    boolean inLowerSlowdownZone = currentPosition <= lowerBound + slowdownRange;
    boolean inUpperSlowdownZone = currentPosition >= upperBound - slowdownRange;
    if (speed < 0 && inLowerSlowdownZone) speed = speed * getLowerSlowdownFactor(currentPosition);
    if (speed > 0 && inUpperSlowdownZone) speed = speed * getUpperSlowdownFactor(currentPosition);
    setSpeed.accept(speed);
  }

  public double getUpperSlowdownFactor(double currentPosition) {
    double remainingDistance = Math.max((upperBound - currentPosition) / slowdownRange, 0);
    if (remainingDistance == 0) return 0;
    return Math.max(remainingDistance, Constants.LimitesEncoders.LimiteFuerzaAceleracion);
  }

  public double getLowerSlowdownFactor(double currentPosition) {
    double remainingDistance = Math.max((currentPosition - lowerBound) / slowdownRange, 0);
    if (remainingDistance == 0) return 0;
    return Math.max(remainingDistance, Constants.LimitesEncoders.LimiteFuerzaAceleracion);
  }
}
