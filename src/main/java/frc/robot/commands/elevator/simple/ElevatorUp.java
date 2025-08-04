package frc.robot.commands.elevator.simple;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.elevator.Elevator;

public class ElevatorUp extends Command {
    private final Elevator elevator;

    public ElevatorUp() {
        elevator = Elevator.getInstance();

        addRequirements(elevator);
    }

    @Override
    public void initialize() {
        elevator.setHeight(1.42);
    }

    @Override
    public boolean isFinished() {
        return elevator.reachedSetpoint();
    }

    @Override
    public void end(boolean interrupted) {
        elevator.setTorqueCurrentFOC(0);
    }
}