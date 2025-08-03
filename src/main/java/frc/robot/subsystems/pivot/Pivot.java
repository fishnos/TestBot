package frc.robot.subsystems.pivot;

import org.littletonrobotics.junction.Logger;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.constants.Constants;
import frc.robot.constants.pivot.PivotConfigBase;
import frc.robot.constants.pivot.PivotConfigComp;
import frc.robot.constants.pivot.PivotConfigSim;

public class Pivot extends SubsystemBase {
    private static Pivot instance = null;
    public static Pivot getInstance() {
        if (instance == null) {
            instance = new Pivot();
        }

        return instance;
    }

    private PivotIO pivotIO;
    private PivotIOInputsAutoLogged pivotIOInputs = new PivotIOInputsAutoLogged();

    private Rotation2d setpoint = Rotation2d.fromDegrees(90);

    private final PivotConfigBase config;

    private Pivot() {
        // IO
        switch (Constants.currentMode) {
            case COMP:
                pivotIO = new PivotIOTalonFX(PivotConfigComp.getInstance());
                config = PivotConfigComp.getInstance();
                break;
            case SIM:
                pivotIO = new PivotIOSim(PivotConfigSim.getInstance());
                config = PivotConfigSim.getInstance();
                break;
            case TEST:
                pivotIO = new PivotIOSim(PivotConfigSim.getInstance());
                config = PivotConfigSim.getInstance();
                break;
            default:
                throw new RuntimeException("Invalid robot mode for Pivot IO");
        }

        setpoint = Rotation2d.fromRadians(Constants.PivotConstants.kSTARTING_ANGLE_RAD);
        CommandScheduler.getInstance().registerSubsystem(this);
    }

    @Override
    public void periodic() {
        pivotIO.updateInputs(pivotIOInputs);
        Logger.processInputs("Pivot", pivotIOInputs);

        pivotIO.setAngle(setpoint);
    }

    public void setTorqueCurrentFOC(double torqueCurrent) {
        pivotIO.setTorqueCurrentFOC(torqueCurrent);
    }

    public void setAngle(Rotation2d angle) {
        setpoint = angle;
        Logger.recordOutput("Pivot/setpoint", angle);
    }

    public void setVoltage(double voltage) {
        pivotIO.setVoltage(voltage);
    }

    public boolean reachedSetpoint() {
        Rotation2d currentPosition = pivotIOInputs.pivotPosition;

        return Math.abs(setpoint.minus(currentPosition).getDegrees()) <= config.getToleranceDegrees();
    }

    public Rotation2d getAngle() {
        return pivotIOInputs.pivotPosition;
    }
}