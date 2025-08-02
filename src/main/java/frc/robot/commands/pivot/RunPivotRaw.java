package frc.robot.commands.pivot;

import org.littletonrobotics.junction.Logger;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.constants.Constants;
import frc.robot.constants.pivot.PivotConfigSim;
import frc.robot.subsystems.pivot.Pivot;

public class RunPivotRaw extends Command {
    private final Pivot pivotSubsystem;
    private final XboxController controller;

    public RunPivotRaw(XboxController controller) {
        this.pivotSubsystem = Pivot.getInstance();
        this.controller = controller;

        addRequirements(pivotSubsystem);
    }

    @Override
    public void execute() {
        double pivotInput = MathUtil.applyDeadband(controller.getLeftY(), Constants.OperatorConstants.kDEADBAND_LEFT_Y);
        Rotation2d desiredRotation = Rotation2d.fromRadians(
            pivotInput * Math.PI    
        );

        pivotSubsystem.setAngle(desiredRotation);

        Logger.recordOutput("Pivot/angleDeg", pivotSubsystem.getAngle().getDegrees());
    }

    @Override
    public boolean isFinished() {
        return false;
    }

    @Override
    public void end(boolean interrupted) {
        pivotSubsystem.setTorqueCurrentFOC(0);
    }
}