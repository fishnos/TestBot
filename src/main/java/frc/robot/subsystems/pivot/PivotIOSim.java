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
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.simulation.SingleJointedArmSim;
import frc.robot.constants.pivot.*;
import frc.robot.constants.Constants;

public class PivotIOSim implements PivotIO {
    private DCMotor pivotFalcon500 = DCMotor.getKrakenX60Foc(1);

    private SingleJointedArmSim pivotArmSim;

    private double prevTimeState = 0;
    private double prevTimeInput = 0;

    private final double kMAX_ANGLE_ROTATIONS;
    private final double kMIN_ANGLE_ROTATIONS;
    private final double kSTARTING_ANGLE_RAD;

    private final PIDController pivotFeedback;
    private final ArmFeedforward pivotFeedforward;

    private final TrapezoidProfile pivotTrapProfile;
    private State currentSetpoint;

    private double prevAngleRad = 0;
    private double pivotAppliedVolts = 0;

    private Rotation2d currentPosition = new Rotation2d();

    public PivotIOSim(PivotConfigBase config) {
        this.pivotArmSim = new SingleJointedArmSim(
            pivotFalcon500,
            config.getMotorToOutputShaftRatio(),
            Constants.PivotConstants.kJKG_METERS_SQUARED,
            Constants.PivotConstants.kPIVOT_LENGTH_METERS,
            Units.rotationsToRadians(config.getMinAngleRotations()),
            Units.rotationsToRadians(config.getMaxAngleRotations()),
            true,
            Units.rotationsToRadians(config.getStartingAngleRotations())
        );

        kMIN_ANGLE_ROTATIONS = config.getMinAngleRotations();
        kMAX_ANGLE_ROTATIONS = config.getMaxAngleRotations();
        kSTARTING_ANGLE_RAD = Units.rotationsToRadians(config.getStartingAngleRotations());

        this.pivotFeedback = new PIDController(
            config.getKP(),
            config.getKI(),
            config.getKD()
        );
        pivotFeedback.enableContinuousInput(-Math.PI, Math.PI);

        this.pivotFeedforward = new ArmFeedforward(
            config.getKS(),
            config.getKG(),
            config.getKV(),
            config.getKA()
        );

        this.pivotTrapProfile = new TrapezoidProfile(new TrapezoidProfile.Constraints(
            Units.rotationsToRadians(config.getMotionMagicCruiseVelocityRotationsPerSec()),
            12.0 / config.getMotionMagicExpoKA() //divide supply voltage, 12V, by 8 (the acceleration constant)
        ));

        currentSetpoint = new State(Units.rotationsToRadians(config.getStartingAngleRotations()), pivotArmSim.getVelocityRadPerSec());
        prevAngleRad = MathUtil.angleModulus(Units.rotationsToRadians(config.getStartingAngleRotations()));
    }

    @Override
    public void updateInputs(PivotIOInputs inputs) {
        double dt = Timer.getTimestamp() - prevTimeInput;
        prevTimeInput = Timer.getTimestamp();

        pivotArmSim.update(dt);

        inputs.pivotVelocityRadPerSec = pivotArmSim.getVelocityRadPerSec();
        inputs.pivotCurrentDrawAmps = pivotArmSim.getCurrentDrawAmps();

        inputs.pivotPosition = Rotation2d.fromRadians(pivotArmSim.getAngleRads());
        prevAngleRad = inputs.pivotPosition.getRadians();

        inputs.pivotAppliedVolts = pivotAppliedVolts;
    }

    @Override
    public void setAngle(Rotation2d desiredPosition) {
        desiredPosition = Rotation2d.fromRadians(
            MathUtil.clamp(
                desiredPosition.getRadians(),
                Units.rotationsToRadians(kMIN_ANGLE_ROTATIONS),
                Units.rotationsToRadians(kMAX_ANGLE_ROTATIONS)
            )
        );

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

        pivotArmSim.setInputVoltage(voltage);
    }

    @Override
    public void setTorqueCurrentFOC(double torqueCurrent) {
        pivotAppliedVolts = torqueCurrent;
        pivotArmSim.setInputVoltage(torqueCurrent);
    }
}
