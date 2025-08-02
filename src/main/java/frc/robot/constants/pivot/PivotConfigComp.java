package frc.robot.constants.pivot;

import edu.wpi.first.math.geometry.Rotation2d;

public class PivotConfigComp implements PivotConfigBase {
    private static PivotConfigComp instance;
    public static PivotConfigComp getInstance() {
        if (instance == null) {
            instance = new PivotConfigComp();
        }

        return instance;
    }
    
    private PivotConfigComp() {}
    
    // CANID
    @Override
    public int getCanID() {
        return 20;
    }

    // Supply current limits
    @Override
    public double getSupplyCurrentLimit() {
        return 55.0;
    }

    @Override
    public double getSupplyCurrentLimitLowerTime() {
        return 1.0;
    }

    @Override
    public double getSupplyCurrentLimitLowerLimit() {
        return 50.0;
    }

    // Stator current limit
    @Override
    public double getStatorCurrentLimit() {
        return 40.0;
    }

    // Peak torque currents
    @Override
    public double getPeakForwardTorqueCurrent() {
        return 40.0;
    }

    @Override
    public double getPeakReverseTorqueCurrent() {
        return -40.0;
    }

    // Characterization / Gains
    @Override
    public double getKS() {
        return 0;
    }

    @Override
    public double getKV() {
        return 0;
    }

    @Override
    public double getKA() {
        return 0.0;
    }

    @Override
    public double getKP() {
        return 1000;
    }

    @Override
    public double getKI() {
        return 0.0;
    }

    @Override
    public double getKD() {
        return 50;
    }

    @Override
    public double getKG() {
        return 0.0; 
    }

    // Motion magic parameters
    @Override
    public double getMotionMagicExpoKA() {
        return 8;
    }

    @Override
    public double getMotionMagicExpoKV() {
        return 11;
    }

    @Override
    public double getMotionMagicCruiseVelocityRotationsPerSec() {
        return 3.0;
    }

    @Override
    public double getToleranceDegrees() {
        return 7.0;
    }
    
    // Neutral mode
    @Override
    public boolean isNeutralModeBrake() {
        return true;
    }

    @Override
    public boolean isInverted() {
        return false;
    }


    // Gear ratio
    @Override
    public double getMotorToOutputShaftRatio() {
        return 45.0;
    }

    @Override
    public double getMaxAngleRotations() {
        return 125.0/360.0;
    }

    @Override 
    public double getMinAngleRotations() {
        return -46.00/360.0;
    }

    @Override 
    public double getStartingAngleRotations() {
        return getMaxAngleRotations();
    } 
}