package frc.robot.subsystems.drivetrain.swerve;

import org.littletonrobotics.junction.Logger;

import com.pathplanner.lib.util.swerve.SwerveSetpointGenerator;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.subsystems.drivetrain.swerve.module.ModuleIO;
import frc.robot.subsystems.drivetrain.swerve.module.ModuleIOInputsAutoLogged;
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

    private final PIDController rotationalVelocityController;

    private SwerveModuleState[] moduleStates = new SwerveModuleState[4];
    private SwerveModulePosition[] modulePositions = new SwerveModulePosition[4];

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

                gyroIO = new GyroIOPigeon2();
                Phoenix6Odometry.getInstance().start();

                break;
            case SIM:
                generalConfig = SwerveModuleGeneralConfigSim.getInstance();

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

                gyroIO = new GyroIOPigeon2();
                Phoenix6Odometry.getInstance().start();

                break;
            case TEST:
                generalConfig = SwerveModuleGeneralConfigSim.getInstance();

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
    }
}
