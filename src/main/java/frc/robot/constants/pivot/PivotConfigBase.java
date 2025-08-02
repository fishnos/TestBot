package frc.robot.constants.pivot;

public interface PivotConfigBase {
    // CAN ID
    public int getCanID();

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
    public double getMotionMagicCruiseVelocityRotationsPerSec();

    public double getToleranceDegrees();
    
    // Neutral mode
    public boolean isNeutralModeBrake();
    public boolean isInverted();


    // Gear ratio
    public double getMotorToOutputShaftRatio();

    // limits
    public double getMaxAngleRotations();
    public double getMinAngleRotations();
    public double getStartingAngleRotations();
}