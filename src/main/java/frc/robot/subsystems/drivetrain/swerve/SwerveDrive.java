package frc.robot.subsystems.drivetrain.swerve;

import java.util.Optional;

import org.littletonrobotics.junction.Logger;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.estimator.SwerveDrivePoseEstimator;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.subsystems.drivetrain.swerve.module.ModuleIO;
import frc.robot.subsystems.drivetrain.swerve.module.ModuleIOInputsAutoLogged;
import frc.robot.subsystems.drivetrain.swerve.module.ModuleIOSim;
import frc.robot.subsystems.drivetrain.swerve.module.ModuleIOTalonFX;
import frc.robot.constants.Constants;
import frc.robot.constants.drivetrain.swerve.module.SwerveModuleGeneralConfigBase;
import frc.robot.constants.drivetrain.swerve.module.SwerveModuleSpecificConfigBase;
import frc.robot.constants.drivetrain.swerve.module.comp.SwerveModuleGeneralConfigComp;
import frc.robot.constants.drivetrain.swerve.module.comp.SwerveModuleSpecificBLConfigComp;
import frc.robot.constants.drivetrain.swerve.module.comp.SwerveModuleSpecificBRConfigComp;
import frc.robot.constants.drivetrain.swerve.module.comp.SwerveModuleSpecificFLConfigComp;
import frc.robot.constants.drivetrain.swerve.module.comp.SwerveModuleSpecificFRConfigComp;
import frc.robot.constants.drivetrain.swerve.module.sim.SwerveModuleGeneralConfigSim;
import frc.robot.constants.drivetrain.swerve.module.sim.SwerveModuleSpecificBLConfigSim;
import frc.robot.constants.drivetrain.swerve.module.sim.SwerveModuleSpecificBRConfigSim;
import frc.robot.constants.drivetrain.swerve.module.sim.SwerveModuleSpecificFLConfigSim;
import frc.robot.constants.drivetrain.swerve.module.sim.SwerveModuleSpecificFRConfigSim;
import frc.robot.subsystems.drivetrain.swerve.gyro.GyroIO;
import frc.robot.subsystems.drivetrain.swerve.gyro.GyroIOInputsAutoLogged;
import frc.robot.subsystems.drivetrain.swerve.gyro.GyroIOPigeon2;

public class SwerveDrive extends SubsystemBase {
    private static SwerveDrive instance = null;
    public static SwerveDrive getInstance() {
        if (instance == null) {
            instance = new SwerveDrive();
        }
        return instance;
    }

    private SwerveDrivePoseEstimator swerveDrivePoseEstimator;

    private final PIDController rotationalVelocityController;

    private SwerveModuleState[] moduleStates = new SwerveModuleState[4];
    private SwerveModulePosition[] modulePositions = new SwerveModulePosition[4];
    private SwerveDriveKinematics swerveKinematics;

    private ChassisSpeeds desiredRobotRelativeSpeeds = new ChassisSpeeds();

    private ModuleIO[] moduleIO = new ModuleIO[4];
    private final ModuleIOInputsAutoLogged[] moduleIOInputs = new ModuleIOInputsAutoLogged[] {
        new ModuleIOInputsAutoLogged(),
        new ModuleIOInputsAutoLogged(),
        new ModuleIOInputsAutoLogged(),
        new ModuleIOInputsAutoLogged()
    };

    private final SwerveModuleSpecificConfigBase[] moduleConfigs;
    private final SwerveModuleGeneralConfigBase generalConfig;

    private final GyroIO gyroIO;
    private GyroIOInputsAutoLogged gyroIOInputs = new GyroIOInputsAutoLogged();

    private double prevTimeInput = 0;

    private SwerveDrive() {
        switch (Constants.currentMode) {
            case COMP:
                generalConfig = SwerveModuleGeneralConfigComp.getInstance();

                moduleConfigs = new SwerveModuleSpecificConfigBase[] {
                    SwerveModuleSpecificFLConfigComp.getInstance(),
                    SwerveModuleSpecificFRConfigComp.getInstance(),
                    SwerveModuleSpecificBLConfigComp.getInstance(),
                    SwerveModuleSpecificBRConfigComp.getInstance()
                };

                moduleIO = new ModuleIO[] {
                    new ModuleIOTalonFX(generalConfig, moduleConfigs[0], 0),
                    new ModuleIOTalonFX(generalConfig, moduleConfigs[1], 1),
                    new ModuleIOTalonFX(generalConfig, moduleConfigs[2], 2),
                    new ModuleIOTalonFX(generalConfig, moduleConfigs[3], 3)
                };

                for (int i = 0; i < 4; i++) {
                    modulePositions[i] = new SwerveModulePosition(0.0, new Rotation2d());
                    moduleStates[i] = new SwerveModuleState(0.0, new Rotation2d());
                }

                swerveKinematics = new SwerveDriveKinematics(
                    moduleConfigs[0].getModulePositionFromCenterMeters(),
                    moduleConfigs[1].getModulePositionFromCenterMeters(),
                    moduleConfigs[2].getModulePositionFromCenterMeters(),
                    moduleConfigs[3].getModulePositionFromCenterMeters()
                );

                gyroIO = new GyroIOPigeon2();

                swerveDrivePoseEstimator = new SwerveDrivePoseEstimator(
                    swerveKinematics,
                    gyroIOInputs.orientation[2],
                    modulePositions, 
                    new Pose2d()
                );

                Phoenix6Odometry.getInstance().start();

                break;
            case SIM:
                generalConfig = SwerveModuleGeneralConfigSim.getInstance();

                moduleConfigs = new SwerveModuleSpecificConfigBase[] {
                    SwerveModuleSpecificFLConfigSim.getInstance(),
                    SwerveModuleSpecificFRConfigSim.getInstance(),
                    SwerveModuleSpecificBLConfigSim.getInstance(),
                    SwerveModuleSpecificBRConfigSim.getInstance()
                };

                moduleIO = new ModuleIO[] {
                    new ModuleIOSim(generalConfig, 0),
                    new ModuleIOSim(generalConfig, 1),
                    new ModuleIOSim(generalConfig, 2),
                    new ModuleIOSim(generalConfig, 3)
                };

                for (int i = 0; i < 4; i++) {
                    modulePositions[i] = new SwerveModulePosition(0.0, new Rotation2d());
                    moduleStates[i] = new SwerveModuleState(0.0, new Rotation2d());
                }

                swerveKinematics = new SwerveDriveKinematics(
                    moduleConfigs[0].getModulePositionFromCenterMeters(),
                    moduleConfigs[1].getModulePositionFromCenterMeters(),
                    moduleConfigs[2].getModulePositionFromCenterMeters(),
                    moduleConfigs[3].getModulePositionFromCenterMeters()
                );

                swerveDrivePoseEstimator = new SwerveDrivePoseEstimator(
                    swerveKinematics,
                    new Rotation2d(),
                    modulePositions,
                    new Pose2d()
                );

                gyroIO = new GyroIOPigeon2();
                Phoenix6Odometry.getInstance().start();

                break;
            case TEST:
                generalConfig = SwerveModuleGeneralConfigSim.getInstance();

                moduleConfigs = new SwerveModuleSpecificConfigBase[] {
                    SwerveModuleSpecificFLConfigSim.getInstance(),
                    SwerveModuleSpecificFRConfigSim.getInstance(),
                    SwerveModuleSpecificBLConfigSim.getInstance(),
                    SwerveModuleSpecificBRConfigSim.getInstance()
                };

                moduleIO = new ModuleIO[] {
                    new ModuleIOSim(generalConfig, 0),
                    new ModuleIOSim(generalConfig, 1),
                    new ModuleIOSim(generalConfig, 2),
                    new ModuleIOSim(generalConfig, 3)
                };

                for (int i = 0; i < 4; i++) {
                    modulePositions[i] = new SwerveModulePosition(0.0, new Rotation2d());
                    moduleStates[i] = new SwerveModuleState(0.0, new Rotation2d());
                }

                swerveKinematics = new SwerveDriveKinematics(
                    moduleConfigs[0].getModulePositionFromCenterMeters(),
                    moduleConfigs[1].getModulePositionFromCenterMeters(),
                    moduleConfigs[2].getModulePositionFromCenterMeters(),
                    moduleConfigs[3].getModulePositionFromCenterMeters()
                );

                swerveDrivePoseEstimator = new SwerveDrivePoseEstimator(
                    swerveKinematics,
                    new Rotation2d(),
                    modulePositions,
                    new Pose2d()
                );

                gyroIO = new GyroIOPigeon2();
                Phoenix6Odometry.getInstance().start();

                break;
            default:
                throw new RuntimeException("Invalid robot mode for SwerveDrive IO");
        }

        rotationalVelocityController = new PIDController(0, 0, 0);

        CommandScheduler.getInstance().registerSubsystem(this);
    }

    public void periodic() {
        double dt = Timer.getTimestamp() - prevTimeInput;
        prevTimeInput = Timer.getTimestamp();

        Phoenix6Odometry.getInstance().stateLock.readLock().lock();
        try {
            gyroIO.updateInputs(gyroIOInputs);
            Logger.processInputs("Gyro", gyroIOInputs);

            for (int i = 0; i < moduleIO.length; i++) {
                moduleIO[i].updateInputs(moduleIOInputs[i]);
                Logger.processInputs("SwerveModule" + i, moduleIOInputs[i]);
            }
        } finally {
            Phoenix6Odometry.getInstance().stateLock.readLock().unlock();
        }

        for (int i = 0; i < modulePositions.length; i++) {
            modulePositions[i] = new SwerveModulePosition(
                moduleIOInputs[i].drivePositionMeters,
                moduleIOInputs[i].steerPositionRadians
            );

            moduleStates[i] = new SwerveModuleState(
                moduleIOInputs[i].driveVelocityMetersPerSec,
                moduleIOInputs[i].steerPositionRadians
            );
        }

        swerveDrivePoseEstimator.update(
            gyroIOInputs.orientation[2],
            modulePositions
        );

        Logger.recordOutput("Swerve/robotPose", swerveDrivePoseEstimator.getEstimatedPosition());
        Logger.recordOutput("Swerve/robotPosition", swerveDrivePoseEstimator.getEstimatedPosition().getTranslation());
        Logger.recordOutput("Swerve/robotRotation", swerveDrivePoseEstimator.getEstimatedPosition().getRotation());
    }

    public void driveRobotRelative(ChassisSpeeds speeds, double dt) {
        Logger.recordOutput("Swerve/desiredRobotRelativeSpeeds", speeds);

        SwerveModuleState[] states = swerveKinematics.toSwerveModuleStates(speeds);
        SwerveDriveKinematics.desaturateWheelSpeeds(states, generalConfig.getDriveMaxVelocityMetersPerSec());

        for (int i = 0; i < moduleStates.length; i++) {
            states[i].optimize(moduleStates[i].angle);

            double stateAccel = (states[i].speedMetersPerSecond - moduleStates[i].speedMetersPerSecond) / dt;
            moduleIO[i].setState(states[i], Optional.of(Double.valueOf(stateAccel)));
        }
    }

    public void driveFieldRelative(ChassisSpeeds speeds, double dt) {
        speeds = ChassisSpeeds.fromFieldRelativeSpeeds(
            speeds,
            gyroIOInputs.orientation[2] //yaw
        );

        driveRobotRelative(speeds, dt);
    }

    public void resetGyro(Rotation2d yaw) {
        gyroIO.resetGyro(yaw);
    }

    public Translation2d getRobotPosition() {
        return swerveDrivePoseEstimator.getEstimatedPosition().getTranslation();
    }

    public Rotation2d getRobotRotation() {
        return swerveDrivePoseEstimator.getEstimatedPosition().getRotation();
    }
}