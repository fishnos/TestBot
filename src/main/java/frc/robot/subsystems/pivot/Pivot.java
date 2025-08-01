package frc.robot.subsystems.pivot;

import org.littletonrobotics.junction.Logger;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

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

    private final Constants.PivotConstants constants;

    private Pivot() {
        // IO
        constants = new Constants.PivotConstants();
        pivotIO = new PivotIOSim(constants);

        setpoint = Rotation2d.fromRotations(constants.kSTARTING_ANGLE_RAD);
        CommandScheduler.getInstance().registerSubsystem(this);
    }

    @Override
    public void periodic() {
        pivotIO.updateInputs(pivotIOInputs);
        Logger.processInputs("Pivot", pivotIOInputs);

        pivotIO.setAngle(setpoint);
    }

    public void setAngle(Rotation2d angle) {
        setpoint = angle;
        Logger.recordOutput("Pivot/setpoint", angle);
    }

    public void setVoltage(double voltage) {
        pivotIO.setVoltage(voltage);
    }

    public Rotation2d getAngle() {
        return pivotIOInputs.pivotPosition;
    }
}