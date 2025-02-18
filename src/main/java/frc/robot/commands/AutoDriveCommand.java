package frc.robot.commands;

import frc.robot.subsystems.MovimientoSubsystem;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj.Timer;

public class AutoDriveCommand extends Command {
    private final MovimientoSubsystem driveSubsystem;
    private final double setpoint;
    private double errorSum = 0;
    private double lastTimestamp = 0;
    private double lastError = 0;

    private final double kP = 0.1;
    private final double kI = 0.05;
    private final double iLimit = 1;
    private final double kD = 0;

    public AutoDriveCommand(MovimientoSubsystem movimientoSubsystem, double setpoint) {
        this.driveSubsystem = movimientoSubsystem;
        this.setpoint = setpoint;
        addRequirements(driveSubsystem);
    }

    @Override
    public void initialize() {
        // Reset encoders
        driveSubsystem.getLeftEncoderPosition();
        driveSubsystem.getRightEncoderPosition();
        errorSum = 0;
        lastTimestamp = Timer.getFPGATimestamp();
        lastError = 0;
    }

    @Override
    public void execute() {
        double leftSensorPosition = driveSubsystem.getLeftEncoderPosition();
        double rightSensorPosition = driveSubsystem.getRightEncoderPosition();

        double error = setpoint - ((leftSensorPosition + rightSensorPosition) / 2);
        double dt = Timer.getFPGATimestamp() - lastTimestamp;

        if (Math.abs(error) < iLimit) {
            errorSum += error * dt;
        }

        double errorRate = (error - lastError) / dt;
        double outputSpeed = kP * error + kI * errorSum + kD * errorRate;

        driveSubsystem.setDriveSpeed(outputSpeed, outputSpeed);

        lastTimestamp = Timer.getFPGATimestamp();
        lastError = error;
    }

    @Override
    public void end(boolean isInterrupted) {
    }

    @Override
    public boolean isFinished() {
        return Math.abs(setpoint - ((driveSubsystem.getLeftEncoderPosition() + driveSubsystem.getRightEncoderPosition()) / 2)) < 0.1;
    }
}
