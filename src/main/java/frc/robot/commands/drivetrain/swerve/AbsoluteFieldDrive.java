package frc.robot.commands.drivetrain.swerve;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.constants.Constants;
import frc.robot.constants.drivetrain.swerve.module.SwerveModuleGeneralConfigBase;
import frc.robot.constants.drivetrain.swerve.module.comp.SwerveModuleGeneralConfigComp;
import frc.robot.constants.drivetrain.swerve.module.sim.SwerveModuleGeneralConfigSim;
import frc.robot.lib.input.XboxController;
import frc.robot.subsystems.drivetrain.swerve.SwerveDrive;

public class AbsoluteFieldDrive extends Command {
    private final XboxController controller;

    private final SwerveModuleGeneralConfigBase generalConfig;
    private final SwerveDrive swerveDrive;
    private int invert = 1;

    private double prevTimeInput = 0;
    private double xInput, yInput, omegaInput;

    public AbsoluteFieldDrive(XboxController controller) {
        this.controller = controller;
        this.swerveDrive = SwerveDrive.getInstance();

        switch (Constants.currentMode) {
            case COMP:
                this.generalConfig = SwerveModuleGeneralConfigComp.getInstance();
                break;
            case SIM:
                this.generalConfig = SwerveModuleGeneralConfigSim.getInstance();
                break;
            case TEST:
                this.generalConfig = SwerveModuleGeneralConfigSim.getInstance();
                break;
            default:
                throw new IllegalStateException("Unexpected value: " + Constants.currentMode);
        }

        addRequirements(swerveDrive);
    }

    @Override
    public void initialize() {
        invert = Constants.shouldInvertField() ? -1 : 1;
        prevTimeInput = Timer.getTimestamp();
    }

    @Override
    public void execute() {
        double dt = Timer.getTimestamp() - prevTimeInput;
        prevTimeInput = Timer.getTimestamp();
        
        xInput = MathUtil.applyDeadband(controller.getLeftY(), Constants.OperatorConstants.kDEADBAND_LEFT_Y);
        yInput = MathUtil.applyDeadband(controller.getLeftX(), Constants.OperatorConstants.kDEADBAND_LEFT_X);
        omegaInput = MathUtil.applyDeadband(controller.getRightX(), Constants.OperatorConstants.kDEADBAND_RIGHT_X);

        ChassisSpeeds driveFieldRelativeSpeeds = new ChassisSpeeds(
            xInput * invert * generalConfig.getDriveMaxVelocityMetersPerSec(),
            yInput * invert * generalConfig.getDriveMaxVelocityMetersPerSec(),
            omegaInput * Units.rotationsToRadians(generalConfig.getSteerMotionMagicCruiseVelocityRotationsPerSec())
        );

        swerveDrive.driveFieldRelative(driveFieldRelativeSpeeds, dt);
    }

    @Override
    public boolean isFinished() {
        return false; // never ends
    }

    @Override
    public void end(boolean interrupted) {}
    
}
