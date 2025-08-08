package frc.robot.subsystems.drivetrain.swerve.module;

import java.util.Optional;

import org.littletonrobotics.junction.Logger;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.system.plant.LinearSystemId;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.math.trajectory.TrapezoidProfile.State;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.simulation.FlywheelSim;
import frc.robot.constants.drivetrain.swerve.module.SwerveModuleGeneralConfigBase;

public class ModuleIOSim implements ModuleIO {
    private final SwerveModuleGeneralConfigBase config;
    private final int moduleID;

    private final DCMotor driveMotor = DCMotor.getKrakenX60(1);
    private final DCMotor steerMotor = DCMotor.getKrakenX60(1);

    private final FlywheelSim driveSim;
    private final FlywheelSim steerSim;

    private final double kDRIVE_JKG_METERS_SQUARED;
    private final double kSTEER_JKG_METERS_SQUARED;

    private final double kDRIVE_MODULE_ROTATIONS_TO_MOTOR_ROTATIONS;
    private final double kSTEER_MODULE_ROTATIONS_TO_MOTOR_ROTATIONS;

    private final SimpleMotorFeedforward driveFeedforward;
    private final SimpleMotorFeedforward steerFeedforward;

    private final SlewRateLimiter driveSlewLimiter;
    private State currentDriveState;

    private final TrapezoidProfile steerTrapProfile;
    private State currentSteerState;
    private State currentSteerSetpoint;

    private final PIDController driveFeedback;
    private final PIDController steerFeedback;

    private double prevDrivePosition = 0;
    private double prevDriveDesiredVeloMps = 0;

    private double prevSteerPosition = 0;

    private double prevTimeInput = 0;
    private double prevTimeState = 0;

    private double currentUnwrappedAngleRad = 0;

    public ModuleIOSim(SwerveModuleGeneralConfigBase config, int moduleID) {
        this.moduleID = moduleID;
        this.config = config;

        kDRIVE_JKG_METERS_SQUARED = 0;
        kSTEER_JKG_METERS_SQUARED = 0;

        kDRIVE_MODULE_ROTATIONS_TO_MOTOR_ROTATIONS = 1 / config.getDriveMotorToOutputShaftRatio();
        kSTEER_MODULE_ROTATIONS_TO_MOTOR_ROTATIONS = 1 / config.getSteerMotorToOutputShaftRatio();

        driveSim = new FlywheelSim(
            LinearSystemId.createFlywheelSystem(
                driveMotor,
                kDRIVE_JKG_METERS_SQUARED,
                kDRIVE_MODULE_ROTATIONS_TO_MOTOR_ROTATIONS
            ), driveMotor
        );
        steerSim = new FlywheelSim(
            LinearSystemId.createFlywheelSystem(
                steerMotor,
                kSTEER_JKG_METERS_SQUARED,
                kSTEER_MODULE_ROTATIONS_TO_MOTOR_ROTATIONS
            ), steerMotor
        );

        driveFeedforward = new SimpleMotorFeedforward(
            config.getDriveKS(),
            config.getDriveKV(),
            config.getDriveKA()
        );
        steerFeedforward = new SimpleMotorFeedforward(
            config.getSteerKS(),
            config.getSteerKV(),
            config.getSteerKA()
        );

        driveFeedback = new PIDController(
            config.getDriveKP(),
            config.getDriveKI(),
            config.getDriveKD()
        );
        steerFeedback = new PIDController(
            config.getSteerKP(),
            config.getSteerKI(),
            config.getSteerKD()
        );

        driveFeedback.setTolerance(0.01);

        steerFeedback.setTolerance(Math.toRadians(1));
        steerFeedback.enableContinuousInput(-Math.PI, Math.PI);

        driveSlewLimiter = new SlewRateLimiter(
            config.getDriveMotionMagicVelocityAccelerationMetersPerSecSec(),
            -config.getDriveMotionMagicVelocityDecelerationMetersPerSecSec(),
            0
        );
        currentDriveState = new State(0, 0);

        steerTrapProfile = new TrapezoidProfile(
            new TrapezoidProfile.Constraints(
                config.getSteerMotionMagicCruiseVelocityRotationsPerSec(),
                12.0 / config.getSteerMotionMagicExpoKA()
            )
        );
        currentSteerState = new State(0, 0);
        currentSteerSetpoint = new State(0, 0);
    }

    @Override
    public void updateInputs(ModuleIOInputs inputs) {
        double dt = Timer.getTimestamp() - prevTimeInput;
        prevTimeInput = Timer.getTimestamp();

        driveSim.update(dt);
        steerSim.update(dt);

        inputs.timestamp = Timer.getTimestamp();

        inputs.driveAppliedVolts = driveSim.getInputVoltage();
        inputs.steerAppliedVolts = steerSim.getInputVoltage();

        inputs.driveCurrentDrawAmps = driveSim.getCurrentDrawAmps();
        inputs.steerCurrentDrawAmps = steerSim.getCurrentDrawAmps();

        inputs.driveVelocityMetersPerSec = driveSim.getAngularVelocityRPM() * Math.PI * 2 * config.getDriveWheelRadiusMeters();
        prevDriveDesiredVeloMps = inputs.driveVelocityMetersPerSec;

        inputs.steerVelocityRadiansPerSec = steerSim.getAngularVelocityRadPerSec();

        inputs.drivePositionMeters += inputs.driveVelocityMetersPerSec * dt;
        inputs.steerPositionRadians = new Rotation2d(
            MathUtil.angleModulus(
                inputs.steerPositionRadians.getRadians() +
                steerSim.getAngularVelocityRadPerSec() * dt
            )
        );

        double angleError = inputs.steerPositionRadians.getRadians() - prevSteerPosition;
        prevSteerPosition = inputs.steerPositionRadians.getRadians();
        prevDrivePosition = inputs.drivePositionMeters;

        angleError = MathUtil.inputModulus(angleError, -Math.PI, Math.PI);

        currentUnwrappedAngleRad += angleError;

        currentDriveState = new State(inputs.drivePositionMeters, inputs.driveVelocityMetersPerSec);
        currentSteerState = new State(inputs.steerPositionRadians.getRadians(), inputs.steerVelocityRadiansPerSec);
        Logger.recordOutput("Swerve/Module" + moduleID + "/currentSteerStateRad", inputs.steerPositionRadians.getRadians());
    }

    @Override
    public void setState(SwerveModuleState state, Optional<Double> feedforwardTorqueCurrent) {
        double dt = Timer.getTimestamp() - prevTimeState;
        prevTimeState = Timer.getTimestamp();

        double driveSetpointMetersPerSec = 
            MathUtil.clamp(
                state.speedMetersPerSecond,
                -config.getDriveMaxVelocityMetersPerSec(),
                config.getDriveMaxVelocityMetersPerSec()
            );

        double driveAccel = (driveSetpointMetersPerSec - prevDriveDesiredVeloMps) / dt;

        if (driveAccel >= 0 && prevDriveDesiredVeloMps >= 0) {
            driveAccel = Math.signum(driveAccel) * Math.min(Math.abs(driveAccel), config.getDriveMotionMagicVelocityAccelerationMetersPerSecSec());
        } 
        else if (driveAccel <= 0 && prevDriveDesiredVeloMps <= 0) {
            driveAccel = Math.signum(driveAccel) * Math.min(Math.abs(driveAccel), config.getDriveMotionMagicVelocityAccelerationMetersPerSecSec());
        }
        else if (driveAccel >= 0 && prevDriveDesiredVeloMps <= 0) {
            driveAccel = Math.signum(driveAccel) * Math.min(Math.abs(driveAccel), config.getDriveMotionMagicVelocityDecelerationMetersPerSecSec());
        }
        else if (driveAccel <= 0 && prevDriveDesiredVeloMps >= 0) {
            driveAccel = Math.signum(driveAccel) * Math.min(Math.abs(driveAccel), config.getDriveMotionMagicVelocityDecelerationMetersPerSecSec());
        }
        else {
            driveAccel = 0;
        }

        driveSetpointMetersPerSec = prevDriveDesiredVeloMps + driveAccel * dt;
        prevDriveDesiredVeloMps = driveSetpointMetersPerSec;

        double desiredAngleRad = MathUtil.angleModulus(state.angle.getRadians());
        double angleError = desiredAngleRad - currentUnwrappedAngleRad;
        angleError = MathUtil.inputModulus(angleError, -Math.PI, Math.PI);

        Logger.recordOutput("Swerve/Module" + moduleID + "/desiredAngleRad", desiredAngleRad);

        currentSteerSetpoint = steerTrapProfile.calculate(
            dt,
            currentSteerSetpoint,
            new State(currentUnwrappedAngleRad + angleError, 0));
        State nextSteerSetpoint = steerTrapProfile.calculate(
            dt * 2.0,
            currentSteerSetpoint,
            new State(currentUnwrappedAngleRad + angleError, 0)
        );
        Logger.recordOutput("Swerve/Module" + moduleID + "/currentSteerSetpointRad", currentSteerSetpoint.position);
        Logger.recordOutput("Swerve/Module" + moduleID + "/currentSteerSetpointRadPerSec", nextSteerSetpoint.velocity);
        
        double driveVolts = 
            driveFeedforward.calculate(driveSetpointMetersPerSec) +
            driveFeedback.calculate(currentDriveState.velocity, driveSetpointMetersPerSec);
        //simply drive forward with feedforward (only accounting for velocity) and then account for error with positional PID
        double steerVolts =
            steerFeedforward.calculateWithVelocities(
                currentSteerSetpoint.velocity, 
                (nextSteerSetpoint.velocity)) +
            steerFeedback.calculate(
                MathUtil.angleModulus(currentSteerState.position),
                MathUtil.angleModulus(currentSteerSetpoint.position)
            );
        Logger.recordOutput("Swerve/Module" + moduleID + "/steerVolts", steerVolts);
        Logger.recordOutput("Swerve/Module" + moduleID + "/driveVolts", driveVolts);

        driveSim.setInputVoltage(driveVolts);
        steerSim.setInputVoltage(steerVolts);
    }

    @Override
    public void setDriveVoltage(double voltage) {
        driveSim.setInputVoltage(voltage);
    }
}
