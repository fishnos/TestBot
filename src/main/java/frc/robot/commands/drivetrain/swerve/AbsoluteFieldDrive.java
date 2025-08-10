package frc.robot.commands.drivetrain.swerve;

import java.util.function.DoubleSupplier;

import org.littletonrobotics.junction.Logger;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.kinematics.ChassisSpeeds; // Class to handle chassis speed calculations.
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command; // Base class for commands.
import frc.robot.constants.Constants;
import frc.robot.constants.drivetrain.swerve.module.SwerveModuleGeneralConfigBase;
import frc.robot.constants.drivetrain.swerve.module.comp.SwerveModuleGeneralConfigComp;
import frc.robot.constants.drivetrain.swerve.module.sim.SwerveModuleGeneralConfigSim;
import frc.robot.lib.input.XboxController;
import frc.robot.subsystems.drivetrain.swerve.SwerveDrive; // Swerve drive subsystem for robot movement.

public class AbsoluteFieldDrive extends Command {

    private final SwerveDrive swerve = SwerveDrive.getInstance();             // Reference to the swerve drive subsystem.
    private final DoubleSupplier vX, vY, heading; // Supplier functions for velocity inputs and heading.
    private int invert = 1;                        // Variable to invert direction based on alliance color.
    
    private final SwerveModuleGeneralConfigBase generalConfigBase;

    private double lastTheta = 0;
    private double lastTime = 0;

    // Constructor to initialize the AbsoluteFieldDrive command.
    public AbsoluteFieldDrive(XboxController xboxDriver) {
        switch (Constants.currentMode) {
            case COMP:
                generalConfigBase = SwerveModuleGeneralConfigComp.getInstance();

                break;
            case SIM:
                generalConfigBase = SwerveModuleGeneralConfigSim.getInstance();

                break;
            default:
                generalConfigBase = SwerveModuleGeneralConfigSim.getInstance();

                break;
        }

        this.vX = () -> -MathUtil.applyDeadband(xboxDriver.getLeftY(), Constants.OperatorConstants.kDEADBAND_LEFT_Y);
        this.vY = () -> -MathUtil.applyDeadband(xboxDriver.getLeftX(), Constants.OperatorConstants.kDEADBAND_LEFT_X);
        this.heading = () -> -MathUtil.applyDeadband(xboxDriver.getRightX(), Constants.OperatorConstants.kDEADBAND_RIGHT_X);

        addRequirements(swerve); // Specify that this command requires the swerve subsystem.
    }

    // Called when the command is initialized.
    @Override
    public void initialize() {
        invert = Constants.shouldInvertField() ? -1 : 1;
        lastTime = Timer.getTimestamp();
    }

    // Called repeatedly while the command is scheduled.
    @Override
    public void execute() {
        double dt = Timer.getTimestamp() - lastTime;

        // Calculate speeds based on input and max speed constants.
        ChassisSpeeds desiredFieldRelativeSpeeds = new ChassisSpeeds(
            vX.getAsDouble() * generalConfigBase.getDriveMaxVelocityMetersPerSec() * invert,
            vY.getAsDouble() * generalConfigBase.getDriveMaxVelocityMetersPerSec() * invert,
            heading.getAsDouble() * Units.rotationsToRadians(generalConfigBase.getSteerMotionMagicCruiseVelocityRotationsPerSec())
        );
        Logger.recordOutput("AbsoluteFieldDrive/desiredFieldRelativeSpeeds", desiredFieldRelativeSpeeds);
        
        double mag = Math.hypot(desiredFieldRelativeSpeeds.vxMetersPerSecond, desiredFieldRelativeSpeeds.vyMetersPerSecond);
        double theta =  Math.atan2(desiredFieldRelativeSpeeds.vyMetersPerSecond, desiredFieldRelativeSpeeds.vxMetersPerSecond);
        double limThetaSec = MathUtil.clamp(Math.toRadians(360 - mag * 120), 0.01, 3000);
        double delta = MathUtil.clamp(theta - lastTheta, -limThetaSec * dt, limThetaSec * dt);
        double limTheta = lastTheta + delta;

        ChassisSpeeds limSpeeds = new ChassisSpeeds();
        limSpeeds.vxMetersPerSecond = Math.cos(limTheta) * mag;
        limSpeeds.vyMetersPerSecond = Math.sin(limTheta) * mag;
        limSpeeds.omegaRadiansPerSecond = desiredFieldRelativeSpeeds.omegaRadiansPerSecond;
        Logger.recordOutput("AbsoluteFieldDrive/limFieldRelativeSpeeds", limSpeeds);

        swerve.driveFieldRelative(desiredFieldRelativeSpeeds); // Drive the robot using the calculated speeds.

        lastTheta = theta;
        lastTime = Timer.getTimestamp();     

    }

    // Called when the command ends or is interrupted.
    @Override
    public void end(boolean interrupted) {
        // Cleanup or reset logic can be added here if necessary.
        // swerve.disableRotationLock();
    }

    // Returns true when the command should end.
    @Override
    public boolean isFinished() {
        return false; // This command runs indefinitely until interrupted.
    }
}