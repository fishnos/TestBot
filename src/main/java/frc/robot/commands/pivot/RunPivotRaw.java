package frc.robot.commands.pivot;

import org.littletonrobotics.junction.Logger;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.Command;
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
        double pivotInput = controller.getLeftY();
        pivotSubsystem.setAngle(Rotation2d.fromRadians(pivotInput * Math.PI));

        Logger.recordOutput("Pivot/angleDeg", pivotSubsystem.getAngle().getDegrees());
    }

    @Override
    public boolean isFinished() {
        return false;
    }

    @Override
    public void end(boolean interrupted) {}
}