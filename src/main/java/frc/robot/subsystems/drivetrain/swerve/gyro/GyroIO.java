package frc.robot.subsystems.drivetrain.swerve.gyro;

import org.littletonrobotics.junction.AutoLog;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;

public interface GyroIO {
    @AutoLog
    class GyroIOInputs {
        public double gyroTemperature = 0;
        public boolean isConnected = false;

        public Rotation2d[] orientation = {
            new Rotation2d(), // roll (x)
            new Rotation2d(), // pitch (y)
            new Rotation2d()  // yaw (z)
        };
        public Rotation2d[] rates = {
            new Rotation2d(), // roll Rate (x)
            new Rotation2d(), // pitch Rate (y)
            new Rotation2d()  // yaw rate (z)
        };
        public Translation2d fieldRelativeAccelMetersPerSec = new Translation2d(); //x and y accel
    }

    public default void updateInputs(GyroIOInputs inputs) {}
    public default void resetGyro(Rotation2d yaw) {}
}
