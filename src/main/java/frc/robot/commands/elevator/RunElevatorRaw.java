package frc.robot.commands.elevator;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.constants.Constants;
import frc.robot.subsystems.elevator.Elevator;

public class RunElevatorRaw extends Command {
    private final Elevator elevator;
    private final XboxController controller;
    
    public RunElevatorRaw(XboxController controller) {
        this.controller = controller;
        this.elevator = Elevator.getInstance();
        
        addRequirements(elevator);
    }

    @Override
    public void execute() {
        double elevatorInput = MathUtil.applyDeadband(controller.getLeftX(), Constants.OperatorConstants.kDEADBAND_LEFT_X);
        double desiredPosition = elevatorInput * 1.42;

        elevator.setHeight(desiredPosition);
    }

    @Override
    public boolean isFinished() {
        return false;
    }

    @Override
    public void end(boolean interrupted) {
        elevator.setTorqueCurrentFOC(0);
    }
}
