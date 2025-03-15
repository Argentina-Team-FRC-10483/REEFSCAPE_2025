package frc.robot.subsystems;

public interface MovableSubsystem {
  void reset();
  double getActual();
  double getTarget();
  void move(double position);
}