package frc.robot.subsystems.elevator;

import frc.robot.constants.elevator.ElevatorConfigBase;

public class ElevatorIOTalonFX implements ElevatorIO {
    private final ElevatorConfigBase config;

    public ElevatorIOTalonFX(ElevatorConfigBase config) {
        this.config = config;
    }

    @Override
    public void updateInputs(ElevatorIOInputs inputs) {

    }

    @Override
    public void setPosition(double position) {

    }

    @Override
    public void setVoltage(double voltage) {

    }

    @Override
    public void setTorqueCurrentFOC(double torqueCurrent) {

    }
}
