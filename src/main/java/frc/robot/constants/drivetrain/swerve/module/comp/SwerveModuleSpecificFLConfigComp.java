package frc.robot.constants.drivetrain.swerve.module.comp;

import edu.wpi.first.math.geometry.Translation2d;
import frc.robot.constants.drivetrain.swerve.module.SwerveModuleSpecificConfigBase;

public class SwerveModuleSpecificFLConfigComp implements SwerveModuleSpecificConfigBase {

    public static SwerveModuleSpecificFLConfigComp instance = null;
    public static SwerveModuleSpecificFLConfigComp getInstance() {
        if (instance == null) {
            instance = new SwerveModuleSpecificFLConfigComp();
        }
        return instance;
    }

    private SwerveModuleSpecificFLConfigComp() {}

    @Override
    public int getDriveCanId() {
        return 2;
    }

    @Override
    public int getSteerCanId() {
        return 1;
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
        return 9;
    }

    @Override
    public double getCancoderOffsetRotations() {
        return 0.611816;
    }

    @Override
    public Translation2d getModulePositionFromCenterMeters() {
        return new Translation2d(0.381, 0.381);
    }
}