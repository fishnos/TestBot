package frc.robot.constants.drivetrain.swerve.module.comp;

import frc.robot.constants.drivetrain.swerve.module.SwerveModuleSpecificConfigBase;

public class SwerveModuleSpecificBRConfigComp implements SwerveModuleSpecificConfigBase {

    public static SwerveModuleSpecificBRConfigComp instance = null;
    public static SwerveModuleSpecificBRConfigComp getInstance() {
        if (instance == null) {
            instance = new SwerveModuleSpecificBRConfigComp();
        }
        return instance;
    }

    private SwerveModuleSpecificBRConfigComp() {}

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