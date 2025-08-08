package frc.robot.subsystems.drivetrain.swerve.module;

import java.util.Optional;

import org.littletonrobotics.junction.AutoLog;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModuleState;

public interface ModuleIO {
    @AutoLog
    class ModuleIOInputs {
        public double timestamp = 0;

        public double drivePositionMeters = 0;
        public double driveVelocityMetersPerSec = 0;

        public Rotation2d steerPositionRadians = new Rotation2d();
        public double steerVelocityRadiansPerSec = 0;

        public double driveCurrentDrawAmps = 0;
        public double driveTemperature = 0;
        public double driveAppliedVolts = 0;

        public double steerCurrentDrawAmps = 0;
        public double steerTemperature = 0;
        public double steerAppliedVolts = 0;

        public boolean driveConnected = true;
        public boolean steerConnected = true;
        public boolean steerEncoderConnected = true;
    }

    public default void updateInputs(ModuleIOInputs inputs) {}
    public default void setDriveVoltage(double voltage) {}
    public default void setState(SwerveModuleState state, Optional<Double> feedforwardTorqueCurrent) {}
}
