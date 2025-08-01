package frc.robot.subsystems.pivot;

import org.littletonrobotics.junction.Logger;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.ArmFeedforward;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.math.trajectory.TrapezoidProfile.State;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.simulation.EncoderSim;
import edu.wpi.first.wpilibj.simulation.SingleJointedArmSim;
import frc.robot.Constants;

public class PivotIOSim implements PivotIO {
    private DCMotor pivotFalcon500 = DCMotor.getKrakenX60Foc(1);

    private SingleJointedArmSim pivotArmSim;

    private double prevTimeState = 0;
    private double prevTimeInput = 0;

    private final PIDController pivotFeedback;
    private final ArmFeedforward pivotFeedforward;

    private final TrapezoidProfile pivotTrapProfile;
    private State currentSetpoint;

    private double prevAngleRad = 0;
    private double pivotAppliedVolts = 0;

    private Rotation2d currentPosition = new Rotation2d();

    public PivotIOSim(Constants.PivotConstants pivotConstants) {
        this.pivotArmSim = new SingleJointedArmSim(
            pivotFalcon500,
            pivotConstants.kPIVOT_MOTOR_TO_OUTPUT_SHAFT_RATIO,
            pivotConstants.kJKG_METERS_SQUARED,
            pivotConstants.kPIVOT_LENGTH_METERS,
            pivotConstants.kMIN_ANGLE_RAD,
            pivotConstants.kMAX_ANGLE_RAD,
            true,
            pivotConstants.kSTARTING_ANGLE_RAD
        );

        this.pivotFeedback = new PIDController(100, 0, 50);
        pivotFeedback.enableContinuousInput(-Math.PI, Math.PI);

        this.pivotFeedforward = new ArmFeedforward(0, 0, 0, 0);

        this.pivotTrapProfile = new TrapezoidProfile(new TrapezoidProfile.Constraints(
            Units.rotationsToRadians(3.0),
            12 / 8.0 //divide supply voltage, 12V, by 8 (the acceleration constant)
        ));

        currentSetpoint = new State(pivotConstants.kSTARTING_ANGLE_RAD, 0);
        prevAngleRad = MathUtil.angleModulus(pivotConstants.kSTARTING_ANGLE_RAD);
    }

    @Override
    public void updateInputs(PivotIOInputs inputs) {
        double dt = Timer.getTimestamp() - prevTimeInput;
        prevTimeInput = Timer.getTimestamp();

        pivotArmSim.update(dt);

        inputs.pivotVelocityRadPerSec = pivotArmSim.getVelocityRadPerSec();
        inputs.pivotCurrentDrawAmps = pivotArmSim.getCurrentDrawAmps();

        inputs.pivotPosition = currentPosition;
        prevAngleRad = inputs.pivotPosition.getRadians();

        inputs.pivotAppliedVolts = pivotAppliedVolts;
    }

    @Override
    public void setAngle(Rotation2d desiredPosition) {
        currentPosition = desiredPosition;
        double dt = Timer.getTimestamp() - prevTimeState;
        prevTimeState = Timer.getTimestamp();

        currentSetpoint = pivotTrapProfile.calculate(dt,
            currentSetpoint,
            new State(desiredPosition.getRadians(),
            0
        ));

        Logger.recordOutput("Pivot/currentProfileSetpoint/position", currentSetpoint.position);

        double voltage =
            pivotFeedforward.calculate(currentSetpoint.position, currentSetpoint.velocity) + 
            pivotFeedback.calculate(prevAngleRad, currentSetpoint.position);
        pivotAppliedVolts = voltage;

        setVoltage(voltage);
    }

    @Override
    public void setVoltage(double voltage) {
        pivotAppliedVolts = voltage;
        pivotArmSim.setInputVoltage(voltage);
    }
}
