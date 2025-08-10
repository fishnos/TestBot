package frc.robot.constants.drivetrain.swerve.module;

import edu.wpi.first.math.geometry.Translation2d;

public interface SwerveModuleSpecificConfigBase {
    public int getDriveCanId();
    public int getSteerCanId();

    public boolean getIsDriveInverted();
    public boolean getIsSteerInverted();

    public boolean isDriveNeutralModeBrake();
    public boolean isSteerNeutralModeBrake();
    
    public int getCancoderCanId();
    public double getCancoderOffsetRotations();

    public Translation2d getModulePositionFromCenterMeters();
}