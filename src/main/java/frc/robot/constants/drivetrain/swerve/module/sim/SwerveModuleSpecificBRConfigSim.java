package frc.robot.constants.drivetrain.swerve.module.sim;

import frc.robot.constants.drivetrain.swerve.module.SwerveModuleSpecificConfigBase;

public class SwerveModuleSpecificBRConfigSim implements SwerveModuleSpecificConfigBase {

    public static SwerveModuleSpecificBRConfigSim instance = null;
    public static SwerveModuleSpecificBRConfigSim getInstance() {
        if (instance == null) {
            instance = new SwerveModuleSpecificBRConfigSim();
        }
        return instance;
    }

    private SwerveModuleSpecificBRConfigSim() {}

    @Override
    public int getDriveCanId() {
        return 6;
    }

    @Override
    public int getSteerCanId() {
        return 5;
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
        return 11;
    }

    @Override
    public double getCancoderOffsetRotations() {
        return 0.529297;
    }
}