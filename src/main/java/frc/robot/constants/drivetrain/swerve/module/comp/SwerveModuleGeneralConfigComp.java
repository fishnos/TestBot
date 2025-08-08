package frc.robot.constants.drivetrain.swerve.module.comp;

import com.ctre.phoenix6.signals.FeedbackSensorSourceValue;
import com.ctre.phoenix6.signals.SensorDirectionValue;

import frc.robot.constants.drivetrain.swerve.module.SwerveModuleGeneralConfigBase;

public class SwerveModuleGeneralConfigComp implements SwerveModuleGeneralConfigBase {

    public static SwerveModuleGeneralConfigComp instance = null;
    public static SwerveModuleGeneralConfigComp getInstance() {
        if (instance == null) {
            instance = new SwerveModuleGeneralConfigComp();
        }
        return instance;
    }

    private SwerveModuleGeneralConfigComp() {}

    @Override
    public String getCanBusName() {
        return "drivetrain";
    }

    @Override
    public double getDriveSupplyCurrentLimit() {
        return 70.0;
    }

    @Override
    public double getDriveSupplyCurrentLimitLowerTime() {
        return 5;
    }

    @Override
    public double getDriveSupplyCurrentLimitLowerLimit() {
        return 65.0;
    }

    @Override
    public double getDriveStatorCurrentLimit() {
        return 60.0;
    }

    @Override
    public double getDrivePeakForwardTorqueCurrent() {
        return 60.0;
    }

    @Override
    public double getDrivePeakReverseTorqueCurrent() {
        return -60.0;
    }

    @Override
    public double getDriveKS() {
        return 3.8; // 3.7-3.8 <4
    }

    @Override
    public double getDriveKV() {
        return 2;
        // return 0.0;

    }

    @Override
    public double getDriveKA() {
        // return 10;
        return 9.8; // 9.8
        // return 0;
    }

    @Override
    public double getDriveKP() {
        return 70;//70
        // return 0;
    }

    @Override
    public double getDriveKI() {
        return 0; //.5
    }

    @Override
    public double getDriveKD() {
        return 1.5;//1.5
        // return 0;
    }

    @Override
    public double getDriveMotionMagicVelocityAccelerationMetersPerSecSec() {
        return 12; // go as fast as possible - ok bc of setpoint gen
    }

    @Override
    public double getDriveMotionMagicVelocityDecelerationMetersPerSecSec() {
        return 12;
    }

    @Override
    public double getDriveMotionMagicVelocityJerkMetersPerSecSecSec() {
        return 280;
    }

    @Override
    public double getDriveMaxVelocityMetersPerSec() {
        return 4.5;
    }

    @Override
    public boolean getIsDriveNeutralModeBrake() {
        return true;
    }

    @Override
    public double getDriveMotorToOutputShaftRatio() {
        return 6.12;
    }

    @Override
    public double getDriveWheelRadiusMeters() {
        return 0.04856143856;
    }

    @Override
    public double getSteerSupplyCurrentLimit() {
        return 60.0;
    }

    @Override
    public double getSteerSupplyCurrentLimitLowerTime() {
        return 1.5;
    }

    @Override
    public double getSteerSupplyCurrentLimitLowerLimit() {
        return 50.0;
    }

    @Override
    public double getSteerStatorCurrentLimit() {
        return 45.0;
    }

    @Override
    public double getSteerPeakForwardTorqueCurrent() {
        return 45.0;
    }

    @Override
    public double getSteerPeakReverseTorqueCurrent() {
        return -45.0;
    }

    @Override
    public double getSteerKS() {
        return 4.5;
    }

    @Override
    public double getSteerKV() {
        return 0;//1.56;
    }

    @Override
    public double getSteerKA() {
        return 0;//3;
    }

    @Override
    public double getSteerKP() {
        return 500;
    }

    @Override
    public double getSteerKI() {
        return 0.0;
    }

    @Override
    public double getSteerKD() {
        return 10.5;
    }

    @Override
    public double getSteerMotionMagicExpoKA() {
        return 0.2;
    }

    @Override
    public double getSteerMotionMagicExpoKV() {
        return 2;
    }

    @Override
    public double getSteerMotionMagicCruiseVelocityRotationsPerSec() {
        return 12;
    }

    @Override
    public boolean getIsSteerNeutralModeBrake() {
        return true;
    }

    @Override
    public double getSteerMotorToOutputShaftRatio() {
        return 21.428;
    }

    @Override
    public double getSteerRotorToSensorRatio() {
        return 21.428;
    }

    @Override
    public FeedbackSensorSourceValue getSteerCancoderFeedbackSensorSource() {
        return FeedbackSensorSourceValue.FusedCANcoder;
    }

    @Override
    public SensorDirectionValue getCancoderSensorDirection() {
        return SensorDirectionValue.CounterClockwise_Positive;
    }

    @Override
    public double getCancoderAbsoluteSensorDiscontinuityPoint() {
        return 0.5;
    }

    @Override
    public double getDriveMinWallCurrent() {
        return 0;
    }

    @Override
    public double getDriveMaxWallVeloMetersPerSec() {
        return 0.02;
    }

    @Override
    public double getDriveKGMetersSquared() {
        return 0.025;
    }

    @Override
    public double getSteerKGMetersSquared() {
        return 0.0008096955;
    }
}