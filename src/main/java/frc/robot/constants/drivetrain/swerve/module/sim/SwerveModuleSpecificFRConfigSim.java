
package frc.robot.constants.drivetrain.swerve.module.sim;

import edu.wpi.first.math.geometry.Translation2d;
import frc.robot.constants.drivetrain.swerve.module.SwerveModuleSpecificConfigBase;

public class SwerveModuleSpecificFRConfigSim implements SwerveModuleSpecificConfigBase {

    public static SwerveModuleSpecificFRConfigSim instance = null;
    public static SwerveModuleSpecificFRConfigSim getInstance() {
        if (instance == null) {
            instance = new SwerveModuleSpecificFRConfigSim();
        }
        return instance;
    }

    private SwerveModuleSpecificFRConfigSim() {}
    
    @Override
    public int getDriveCanId() {
        return 4;
    }

    @Override
    public int getSteerCanId() {
        return 3;
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
        return 10;
    }

    @Override
    public double getCancoderOffsetRotations() {
        return -0.750977;
    }

    @Override
    public Translation2d getModulePositionFromCenterMeters() {
        return new Translation2d(0.381, -0.381);
    }
}