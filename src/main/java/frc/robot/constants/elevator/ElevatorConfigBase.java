
package frc.robot.constants.elevator;

public interface ElevatorConfigBase {
    // CAN ID
    public int getCanID1();
    public int getCanID2();

    // Supply current limits
    public double getSupplyCurrentLimit();
    public double getSupplyCurrentLimitLowerTime();
    public double getSupplyCurrentLimitLowerLimit();

    // Stator current limit
    public double getStatorCurrentLimit();

    // Peak torque currents
    public double getPeakForwardTorqueCurrent();
    public double getPeakReverseTorqueCurrent();

    // Characterization / Gains
    public double getKS();
    public double getKV();
    public double getKA();
    public double getKP();
    public double getKI();
    public double getKD();
    public double getKG();

    // Motion magic parameters
    public double getMotionMagicExpoKA();
    public double getMotionMagicExpoKV();
    public double getMotionMagicCruiseVelocityMetersPerSec();
    public double getMotionMagicJerkMetersPerSecSecSec();

    // Neutral mode
    public boolean isNeutralModeBrake();

    public boolean isM1Inverted();
    public boolean isM2Inverted();

    // Gear ratio
    public double getMotorToOutputShaftRatio();

    public double getMaxHeightMeters();
    public double getMinHeightMeters();

    public double getToleranceMeters();
    public double getToleranceMetersPerSec();
}