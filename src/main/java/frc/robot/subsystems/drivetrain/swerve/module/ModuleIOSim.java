package frc.robot.subsystems.drivetrain.swerve.module;

import java.util.Optional;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.system.plant.LinearSystemId;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.math.trajectory.TrapezoidProfile.State;
import edu.wpi.first.wpilibj.simulation.FlywheelSim;
import edu.wpi.first.wpilibj.simulation.LinearSystemSim;
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
    private final State currentDriveSetpoint;

    private final TrapezoidProfile steerTrapProfile;
    private final State currentSteerSetpoint;

    private final PIDController driveFeedback;
    private final PIDController steerFeedback;

    private double prevDrivePosition = 0;
    private double prevSteerPosition = 0;

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
        currentDriveSetpoint = new State(0, 0);

        steerTrapProfile = new TrapezoidProfile(
            new TrapezoidProfile.Constraints(
                config.getSteerMotionMagicCruiseVelocityRotationsPerSec(),
                12.0 / config.getSteerMotionMagicExpoKA()
            )
        );
        currentSteerSetpoint = new State(0, 0);
    }

    @Override
    public void updateInputs(ModuleIOInputs inputs) {

    }

    @Override
    public void setDriveVoltage(double voltage) {

    }

    @Override
    public void setState(SwerveModuleState state, Optional<Double> feedforwardTorqueCurrent) {
        
    }

}
