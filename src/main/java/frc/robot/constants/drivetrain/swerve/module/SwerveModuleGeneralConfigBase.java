package frc.robot.constants.drivetrain.swerve.module;

import com.ctre.phoenix6.signals.FeedbackSensorSourceValue;
import com.ctre.phoenix6.signals.SensorDirectionValue;

public interface SwerveModuleGeneralConfigBase {
    public String getCanBusName();

    public double getDriveSupplyCurrentLimit();
    public double getDriveSupplyCurrentLimitLowerTime();
    public double getDriveSupplyCurrentLimitLowerLimit();

    public double getDriveStatorCurrentLimit();

    public double getDriveKGMetersSquared();

    public double getDrivePeakForwardTorqueCurrent();
    public double getDrivePeakReverseTorqueCurrent();

    public double getDriveKS();
    public double getDriveKV();
    public double getDriveKA();
    public double getDriveKP();
    public double getDriveKI();
    public double getDriveKD();

    public double getDriveMaxVelocityMetersPerSec();

    public double getDriveMotionMagicVelocityAccelerationMetersPerSecSec();
    public double getDriveMotionMagicVelocityDecelerationMetersPerSecSec();
    public double getDriveMotionMagicVelocityJerkMetersPerSecSecSec();

    public boolean getIsDriveNeutralModeBrake();

    public double getDriveMotorToOutputShaftRatio();
    public double getDriveWheelRadiusMeters();

    public double getSteerSupplyCurrentLimit();
    public double getSteerSupplyCurrentLimitLowerTime();
    public double getSteerSupplyCurrentLimitLowerLimit();

    public double getSteerStatorCurrentLimit();

    public double getSteerKGMetersSquared();

    public double getSteerPeakForwardTorqueCurrent();
    public double getSteerPeakReverseTorqueCurrent();

    public double getSteerKS();
    public double getSteerKV();
    public double getSteerKA();
    public double getSteerKP();
    public double getSteerKI();
    public double getSteerKD();

    public double getSteerMotionMagicExpoKA();
    public double getSteerMotionMagicExpoKV();
    public double getSteerMotionMagicCruiseVelocityRotationsPerSec();

    public boolean getIsSteerNeutralModeBrake();

    public double getSteerMotorToOutputShaftRatio();
    public double getSteerRotorToSensorRatio();

    public FeedbackSensorSourceValue getSteerCancoderFeedbackSensorSource();

    public SensorDirectionValue getCancoderSensorDirection();
    public double getCancoderAbsoluteSensorDiscontinuityPoint();

    public double getDriveMinWallCurrent();
    public double getDriveMaxWallVeloMetersPerSec();
}