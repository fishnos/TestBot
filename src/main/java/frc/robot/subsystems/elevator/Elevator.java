package frc.robot.subsystems.elevator;

import org.littletonrobotics.junction.Logger;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.constants.Constants;
import frc.robot.constants.elevator.ElevatorConfigBase;
import frc.robot.constants.elevator.ElevatorConfigComp;
import frc.robot.constants.elevator.ElevatorConfigSim;

public class Elevator extends SubsystemBase {
    private static Elevator instance = null;
    public static Elevator getInstance() {
        if (instance == null) {
            instance = new Elevator();
        }
        return instance;
    }

    private ElevatorIO elevatorIO;
    private ElevatorIOInputsAutoLogged elevatorIOInputs = new ElevatorIOInputsAutoLogged();

    private final ElevatorConfigBase config;

    private double setpoint = 0;

    public Elevator() {
        switch (Constants.currentMode) {
            case COMP:
                config = ElevatorConfigComp.getInstance();
                elevatorIO = new ElevatorIOTalonFX(config);
                break;
            case SIM:
                config = ElevatorConfigSim.getInstance();
                elevatorIO = new ElevatorIOSim(config);
                break;
            case TEST:
                config = ElevatorConfigSim.getInstance();
                elevatorIO = new ElevatorIOSim(config);
                break;
            default:
                throw new RuntimeException("Invalid robot mode for Elevator IO");
        }

        CommandScheduler.getInstance().registerSubsystem(this);
    }

    @Override
    public void periodic() {
        elevatorIO.updateInputs(elevatorIOInputs);
        Logger.processInputs("Elevator", elevatorIOInputs);

        elevatorIO.setPosition(setpoint);
    }

    public void setHeight(double heightMeters) {
        setpoint = heightMeters;
        Logger.recordOutput("Elevator/desiredMeters", setpoint);
    }

    public void setVoltage(double voltage) {
        elevatorIO.setVoltage(voltage);
    }

    public void setTorqueCurrentFOC(double torqueCurrent) {
        elevatorIO.setTorqueCurrentFOC(torqueCurrent);
    }

    public boolean reachedSetpoint() {
        return (
            MathUtil.isNear(elevatorIOInputs.elevatorPositionMeters, setpoint, config.getToleranceMeters()) && 
            MathUtil.isNear(elevatorIOInputs.elevatorVelocityMetersPerSec, 0, config.getToleranceMetersPerSec())
        );
    }
}
