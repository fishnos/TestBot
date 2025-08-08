package frc.robot.constants.drivetrain.swerve.module;

public interface SwerveModuleSpecificConfigBase {
    public int getDriveCanId();
    public int getSteerCanId();

    public boolean getIsDriveInverted();
    public boolean getIsSteerInverted();

    public boolean isDriveNeutralModeBrake();
    public boolean isSteerNeutralModeBrake();
    
    public int getCancoderCanId();
    public double getCancoderOffsetRotations();
}