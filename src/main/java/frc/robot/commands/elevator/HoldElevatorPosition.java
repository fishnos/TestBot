package frc.robot.commands.elevator;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.elevator.Elevator;

public class HoldElevatorPosition extends Command {
    private final Elevator elevator;

    public HoldElevatorPosition() {
        elevator = Elevator.getInstance();

        addRequirements(elevator);
    }

    @Override
    public void execute() {
        elevator.setHeight(elevator.getCurrentHeight());
    }

    @Override
    public boolean isFinished() {
        return false;
    }

    @Override
    public void end(boolean interrupted) {}
}
