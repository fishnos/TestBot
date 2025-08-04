package frc.robot.subsystems.elevator;

import org.littletonrobotics.junction.AutoLog;

public interface ElevatorIO {
    @AutoLog
    class ElevatorIOInputs {
        public double elevatorVelocityMetersPerSec = 0;
        public double elevatorPositionMeters = 0;

        public double elevatorAppliedVolts1 = 0;
        public double elevatorAppliedVolts2 = 0;

        public double elevatorTemperature1 = 0;
        public double elevatorTemperature2 = 0;

        public double elevatorCurrentDrawAmps1 = 0;
        public double elevatorCurrentDrawAmps2 = 0;

        public boolean elevatorMotor1Connected = true;
        public boolean elevatorMotor2Connected = true;
    }

    public default void updateInputs(ElevatorIOInputs inputs) {}
    public default void setVoltage(double voltage) {}
    public default void setTorqueCurrentFOC(double torqueCurrent) {}
    public default void setPosition(double positionMeters) {}
}
