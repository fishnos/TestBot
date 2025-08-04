package frc.robot.subsystems.elevator;

import static edu.wpi.first.units.Units.Amps;
import static edu.wpi.first.units.Units.Fahrenheit;
import static edu.wpi.first.units.Units.Rotation;
import static edu.wpi.first.units.Units.RotationsPerSecond;
import static edu.wpi.first.units.Units.Volts;

import org.littletonrobotics.junction.Logger;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.MotionMagicExpoTorqueCurrentFOC;
import com.ctre.phoenix6.controls.StrictFollower;
import com.ctre.phoenix6.controls.TorqueCurrentFOC;
import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.GravityTypeValue;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import com.ctre.phoenix6.signals.StaticFeedforwardSignValue;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.filter.Debouncer;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Current;
import edu.wpi.first.units.measure.Temperature;
import edu.wpi.first.units.measure.Voltage;
import frc.robot.constants.elevator.ElevatorConfigBase;
import frc.robot.lib.util.PhoenixUtil;

public class ElevatorIOTalonFX implements ElevatorIO {
    private final ElevatorConfigBase config;
    private final TalonFXConfiguration elevatorConfig;

    private final double kSTARTING_HEIGHT_METERS;
    private final double kMIN_HEIGHT_METERS;
    private final double kMAX_HEIGHT_METERS;

    private final TalonFX elevatorMotor1;
    private final TalonFX elevatorMotor2;

    private final StatusSignal<Angle> elevatorPositionStatusSignal1;
    private final StatusSignal<Angle> elevatorPositionStatusSignal2;

    private final StatusSignal<AngularVelocity> elevatorVelocityStatusSignal1;
    private final StatusSignal<AngularVelocity> elevatorVelocityStatusSignal2;

    private final StatusSignal<Current> elevatorTorqueCurrentStatusSignal1;
    private final StatusSignal<Current> elevatorTorqueCurrentStatusSignal2;

    private final StatusSignal<Voltage> elevatorAppliedVoltStatusSignal1;
    private final StatusSignal<Voltage> elevatorAppliedVoltStatusSignal2;

    private final StatusSignal<Double> elevatorDutyCycleStatusSignal1;
    private final StatusSignal<Double> elevatorDutyCycleStatusSignal2;

    private final StatusSignal<Temperature> elevatorTemperatureStatusSignal1;
    private final StatusSignal<Temperature> elevatorTemperatureStatusSignal2;

    private final StatusSignal<Double> elevatorClosedLoopReferenceStatusSignal1;
    private final StatusSignal<Double> elevatorClosedLoopReferenceStatusSignal2;

    private final StatusSignal<Double> elevatorClosedLoopOutputStatusSignal1;
    private final StatusSignal<Double> elevatorClosedLoopOutputStatusSignal2;

    private final MotionMagicExpoTorqueCurrentFOC elevatorPositionRequest = new MotionMagicExpoTorqueCurrentFOC(0).withSlot(0);
    private final VoltageOut elevatorVoltageRequest = new VoltageOut(0).withEnableFOC(false);
    private final TorqueCurrentFOC elevatorTorqueRequest = new TorqueCurrentFOC(0);

    private final Debouncer connectedDebouncer1 = new Debouncer(0.25, Debouncer.DebounceType.kBoth);
    private final Debouncer connectedDebouncer2 = new Debouncer(0.25, Debouncer.DebounceType.kBoth);

    public ElevatorIOTalonFX(ElevatorConfigBase config) {
        elevatorConfig = new TalonFXConfiguration();
        this.config = config;

        kSTARTING_HEIGHT_METERS = config.getMinHeightMeters();
        kMIN_HEIGHT_METERS = config.getMinHeightMeters();
        kMAX_HEIGHT_METERS = config.getMaxHeightMeters();

        elevatorMotor1 = new TalonFX(config.getCanID1(), "elevator1");
        PhoenixUtil.tryUntilOk(5, () ->
            elevatorMotor1.getConfigurator().apply(
                elevatorConfig.withMotorOutput(
                    elevatorConfig.MotorOutput.withInverted(
                        config.isM1Inverted() ?
                            InvertedValue.Clockwise_Positive :
                            InvertedValue.CounterClockwise_Positive
                    )
                ),
                0.25
            ));

        elevatorMotor2 = new TalonFX(config.getCanID2(), "elevator2");
        PhoenixUtil.tryUntilOk(5, () ->
            elevatorMotor2.getConfigurator().apply(
                elevatorConfig.withMotorOutput(
                    elevatorConfig.MotorOutput.withInverted(
                        config.isM2Inverted() ?
                            InvertedValue.Clockwise_Positive :
                            InvertedValue.CounterClockwise_Positive
                    )
                ),
                0.25
            ));

        elevatorConfig.Slot0.kA = config.getKA();
        elevatorConfig.Slot0.kV = config.getKV();
        elevatorConfig.Slot0.kS = config.getKS();
        elevatorConfig.Slot0.kP = config.getKP();
        elevatorConfig.Slot0.kI = config.getKI();
        elevatorConfig.Slot0.kD = config.getKD();

        elevatorConfig.Slot0.GravityType = GravityTypeValue.Elevator_Static;
        elevatorConfig.Slot0.StaticFeedforwardSign = StaticFeedforwardSignValue.UseVelocitySign;

        elevatorConfig.Feedback.SensorToMechanismRatio = config.getMotorToOutputShaftRatio();

        elevatorConfig.CurrentLimits.StatorCurrentLimit = config.getStatorCurrentLimit();
        elevatorConfig.CurrentLimits.StatorCurrentLimitEnable = true;
        elevatorConfig.CurrentLimits.SupplyCurrentLimit = config.getSupplyCurrentLimit();
        elevatorConfig.CurrentLimits.SupplyCurrentLimitEnable = true;
        elevatorConfig.CurrentLimits.SupplyCurrentLowerTime = config.getSupplyCurrentLimitLowerTime();
        elevatorConfig.CurrentLimits.SupplyCurrentLowerLimit = config.getSupplyCurrentLimitLowerLimit();

        elevatorConfig.TorqueCurrent.PeakForwardTorqueCurrent = config.getPeakForwardTorqueCurrent();
        elevatorConfig.TorqueCurrent.PeakReverseTorqueCurrent = config.getPeakReverseTorqueCurrent();

        elevatorPositionStatusSignal1 = elevatorMotor1.getPosition().clone();
        elevatorPositionStatusSignal2 = elevatorMotor2.getPosition().clone();

        elevatorVelocityStatusSignal1 = elevatorMotor1.getVelocity().clone();
        elevatorVelocityStatusSignal2 = elevatorMotor2.getVelocity().clone();

        elevatorTorqueCurrentStatusSignal1 = elevatorMotor1.getSupplyCurrent().clone();
        elevatorTorqueCurrentStatusSignal2 = elevatorMotor2.getSupplyCurrent().clone();

        elevatorAppliedVoltStatusSignal1 = elevatorMotor1.getMotorVoltage().clone();
        elevatorAppliedVoltStatusSignal2 = elevatorMotor2.getMotorVoltage().clone();

        elevatorTemperatureStatusSignal1 = elevatorMotor1.getDeviceTemp().clone();
        elevatorTemperatureStatusSignal2 = elevatorMotor2.getDeviceTemp().clone();

        elevatorClosedLoopReferenceStatusSignal1 = elevatorMotor1.getClosedLoopReference().clone();
        elevatorClosedLoopReferenceStatusSignal2 = elevatorMotor2.getClosedLoopReference().clone();

        elevatorClosedLoopOutputStatusSignal1 = elevatorMotor1.getClosedLoopOutput().clone();
        elevatorClosedLoopOutputStatusSignal2 = elevatorMotor2.getClosedLoopOutput().clone();

        elevatorDutyCycleStatusSignal1 = elevatorMotor1.getDutyCycle().clone();
        elevatorDutyCycleStatusSignal2 = elevatorMotor2.getDutyCycle().clone();

        elevatorConfig.MotorOutput.NeutralMode = 
            config.isNeutralModeBrake() ?
                NeutralModeValue.Brake :
                NeutralModeValue.Coast;

        elevatorConfig.FutureProofConfigs = true;

        BaseStatusSignal.setUpdateFrequencyForAll(
            70,
            elevatorAppliedVoltStatusSignal1,
            elevatorAppliedVoltStatusSignal2,

            elevatorTorqueCurrentStatusSignal1,
            elevatorTorqueCurrentStatusSignal2,

            elevatorClosedLoopOutputStatusSignal1,
            elevatorClosedLoopOutputStatusSignal2,
            elevatorClosedLoopReferenceStatusSignal1,
            elevatorClosedLoopReferenceStatusSignal2,
            elevatorDutyCycleStatusSignal1,
            elevatorDutyCycleStatusSignal2,

            elevatorPositionStatusSignal1,
            elevatorPositionStatusSignal2,
            elevatorVelocityStatusSignal1,
            elevatorVelocityStatusSignal2,

            elevatorTemperatureStatusSignal1,
            elevatorTemperatureStatusSignal2
        );

        elevatorMotor1.optimizeBusUtilization();
        elevatorMotor2.optimizeBusUtilization();

        elevatorMotor2.setControl(new StrictFollower(config.getCanID1()).withUpdateFreqHz(1000));
    }

    @Override
    public void updateInputs(ElevatorIOInputs inputs) {
        inputs.elevatorMotor1Connected = 
            connectedDebouncer1.calculate(
                BaseStatusSignal.refreshAll(
                    elevatorAppliedVoltStatusSignal1,

                    elevatorTorqueCurrentStatusSignal1,

                    elevatorClosedLoopOutputStatusSignal1,
                    elevatorClosedLoopReferenceStatusSignal1,
                    elevatorDutyCycleStatusSignal1,

                    elevatorPositionStatusSignal1,
                    elevatorVelocityStatusSignal1,

                    elevatorTemperatureStatusSignal1
                ).isOK()
            );

        inputs.elevatorMotor2Connected = 
            connectedDebouncer2.calculate(
                BaseStatusSignal.refreshAll(
                    elevatorAppliedVoltStatusSignal2,

                    elevatorTorqueCurrentStatusSignal2,

                    elevatorClosedLoopOutputStatusSignal2,
                    elevatorClosedLoopReferenceStatusSignal2,
                    elevatorDutyCycleStatusSignal2,

                    elevatorPositionStatusSignal2,
                    elevatorVelocityStatusSignal2,

                    elevatorTemperatureStatusSignal2
                ).isOK()
            );
        
        inputs.elevatorAppliedVolts1 = elevatorAppliedVoltStatusSignal1.getValue().in(Volts);
        inputs.elevatorAppliedVolts2 = elevatorAppliedVoltStatusSignal2.getValue().in(Volts);

        inputs.elevatorCurrentDrawAmps1 = elevatorTorqueCurrentStatusSignal1.getValue().in(Amps);
        inputs.elevatorCurrentDrawAmps2 = elevatorTorqueCurrentStatusSignal2.getValue().in(Amps);

        inputs.elevatorPositionMeters = 
            (BaseStatusSignal.getLatencyCompensatedValue(elevatorPositionStatusSignal1, elevatorVelocityStatusSignal1).in(Rotation) +
            BaseStatusSignal.getLatencyCompensatedValue(elevatorPositionStatusSignal2, elevatorVelocityStatusSignal2).in(Rotation)) / 2.0;

        inputs.elevatorVelocityMetersPerSec = 
            (elevatorVelocityStatusSignal1.getValue().in(RotationsPerSecond) + 
            elevatorVelocityStatusSignal2.getValue().in(RotationsPerSecond)) / 2.0;

        inputs.elevatorTemperature1 = elevatorTemperatureStatusSignal1.getValue().in(Fahrenheit);
        inputs.elevatorTemperature2 = elevatorTemperatureStatusSignal2.getValue().in(Fahrenheit);

        Logger.recordOutput("Elevator/closedLoopOutputMotor1", elevatorClosedLoopOutputStatusSignal1.getValueAsDouble());
        Logger.recordOutput("Elevator/closedLoopOutputMotor2", elevatorClosedLoopOutputStatusSignal2.getValueAsDouble());

        Logger.recordOutput("Elevator/closedLoopReferenceMotor1", elevatorClosedLoopReferenceStatusSignal1.getValueAsDouble());
        Logger.recordOutput("Elevator/closedLoopReferenceMotor2", elevatorClosedLoopReferenceStatusSignal1.getValueAsDouble());
    }

    @Override
    public void setPosition(double position) {
        elevatorMotor1.setControl(
            elevatorPositionRequest.withPosition(
                MathUtil.clamp(
                    position,
                    kMIN_HEIGHT_METERS,
                    kMAX_HEIGHT_METERS
                )
            )
        );
    }

    @Override
    public void setVoltage(double voltage) {
        elevatorMotor1.setControl(
            elevatorVoltageRequest.withOutput(voltage)
        );
    }

    @Override
    public void setTorqueCurrentFOC(double torqueCurrent) {
        elevatorMotor1.setControl(
            elevatorTorqueRequest.withOutput(torqueCurrent)
        );
    }
}
