package frc.robot.constants.drivetrain.swerve.module.sim;

import frc.robot.constants.drivetrain.swerve.module.SwerveModuleSpecificConfigBase;

public class SwerveModuleSpecificBLConfigSim implements SwerveModuleSpecificConfigBase {

    public static SwerveModuleSpecificBLConfigSim instance = null;
    public static SwerveModuleSpecificBLConfigSim getInstance() {
        if (instance == null) {
            instance = new SwerveModuleSpecificBLConfigSim();
        }
        return instance;
    }

    private SwerveModuleSpecificBLConfigSim() {}
    
    @Override
    public int getDriveCanId() {
        return 8;
    }

    @Override
    public int getSteerCanId() {
        return 7;
    }

    @Override
    public boolean getIsDriveInverted() {
        return false;
    }

    @Override
    public boolean getIsSteerInverted() {
        return true;
    }

    @Override
    public boolean isDriveNeutralModeBrake() {
        return true;
    }

    @Override
    public boolean isSteerNeutralModeBrake() {
        return true;
    }

    @Override
    public int getCancoderCanId() {
        return 12;
    }

    @Override
    public double getCancoderOffsetRotations() {
        return -0.596191;
    }
}