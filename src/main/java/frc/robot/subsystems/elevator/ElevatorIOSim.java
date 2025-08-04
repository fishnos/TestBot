package frc.robot.subsystems.elevator;

import org.littletonrobotics.junction.Logger;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.ElevatorFeedforward;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.math.trajectory.TrapezoidProfile.State;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.simulation.ElevatorSim;
import frc.robot.constants.elevator.ElevatorConfigBase;

public class ElevatorIOSim implements ElevatorIO {
    private final DCMotor elevatorFalcon500 = DCMotor.getKrakenX60Foc(2);

    private final ElevatorSim elevatorSim;

    private final TrapezoidProfile elevatorTrapProfile;
    private TrapezoidProfile.State currentSetpoint;

    private final PIDController elevatorFeedback;
    private final ElevatorFeedforward elevatorFeedforward;

    private final double kMAX_HEIGHT_METERS;
    private final double kMIN_HEIGHT_METERS;
    private final double kSTARTING_HEIGHT_METERS;
    private final double kJKG_METERS_SQUARED;
    private final double kMOTOR_TO_OUTPUT_SHAFT_RATIO;
    private final double kDRUM_RADIUS_METERS;

    private double prevHeightMeters = 0;
    private double elevatorAppliedVolts = 0;

    private double prevTimeState = 0;
    private double prevTimeInput = 0;

    public ElevatorIOSim(ElevatorConfigBase config) {
        kSTARTING_HEIGHT_METERS = config.getMinHeightMeters();
        kMAX_HEIGHT_METERS = config.getMaxHeightMeters();
        kMIN_HEIGHT_METERS = config.getMinHeightMeters();
        kMOTOR_TO_OUTPUT_SHAFT_RATIO = config.getMotorToOutputShaftRatio();
        kDRUM_RADIUS_METERS = 0.05;
        kJKG_METERS_SQUARED = 0.1;

        elevatorSim = new ElevatorSim(
            elevatorFalcon500,
            kMOTOR_TO_OUTPUT_SHAFT_RATIO,
            kJKG_METERS_SQUARED,
            kDRUM_RADIUS_METERS,
            kMIN_HEIGHT_METERS,
            kMAX_HEIGHT_METERS,
            true,
            kSTARTING_HEIGHT_METERS
        );

        elevatorFeedback = new PIDController(
            config.getKP(),
            config.getKI(),
            config.getKD()
        );
        elevatorFeedback.setTolerance(0.01, 0.02);

        elevatorFeedforward = new ElevatorFeedforward(
            config.getKS(),
            config.getKV(),
            config.getKA()
        );

        elevatorTrapProfile = new TrapezoidProfile(new TrapezoidProfile.Constraints(
            config.getMotionMagicCruiseVelocityMetersPerSec(),
            12.0 / config.getMotionMagicExpoKA()
        ));

        currentSetpoint = new State(kSTARTING_HEIGHT_METERS, elevatorSim.getVelocityMetersPerSecond());
    }

    @Override
    public void updateInputs(ElevatorIOInputs inputs) {
        double dt = Timer.getTimestamp() - prevTimeInput;
        prevTimeInput = Timer.getTimestamp();

        elevatorSim.update(dt);

        inputs.elevatorAppliedVolts1 = elevatorAppliedVolts;
        inputs.elevatorAppliedVolts2 = elevatorAppliedVolts;

        inputs.elevatorPositionMeters = elevatorSim.getPositionMeters();
        this.prevHeightMeters = inputs.elevatorPositionMeters;

        inputs.elevatorVelocityMetersPerSec = elevatorSim.getVelocityMetersPerSecond();

        inputs.elevatorCurrentDrawAmps1 = elevatorSim.getCurrentDrawAmps();
        inputs.elevatorCurrentDrawAmps2 = elevatorSim.getCurrentDrawAmps();
    }

    @Override
    public void setVoltage(double voltage) {
        elevatorAppliedVolts = voltage;
        elevatorSim.setInputVoltage(voltage);
    }

    @Override
    public void setTorqueCurrentFOC(double torqueCurrent) {
        elevatorAppliedVolts = torqueCurrent;
        elevatorSim.setInputVoltage(torqueCurrent);
    }

    @Override
    public void setPosition(double positionMeters) {
        double dt = Timer.getTimestamp() - prevTimeState;
        prevTimeState = Timer.getTimestamp();

        positionMeters = MathUtil.clamp(
            positionMeters,
            kMIN_HEIGHT_METERS, 
            kMAX_HEIGHT_METERS
        );

        currentSetpoint = elevatorTrapProfile.calculate(
            dt, 
            currentSetpoint,
            new State(positionMeters, 0)
        );

        Logger.recordOutput("Elevator/currentProfileSetpoint/position", currentSetpoint.position);

        @SuppressWarnings("removal")
        double voltage = 
            elevatorFeedforward.calculate(currentSetpoint.position, currentSetpoint.velocity) +
            elevatorFeedback.calculate(prevHeightMeters, currentSetpoint.position);
        elevatorAppliedVolts = voltage;
    }
}
