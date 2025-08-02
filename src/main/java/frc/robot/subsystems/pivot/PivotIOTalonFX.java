package frc.robot.subsystems.pivot;

import static edu.wpi.first.units.Units.Amps;
import static edu.wpi.first.units.Units.Fahrenheit;
import static edu.wpi.first.units.Units.Radians;
import static edu.wpi.first.units.Units.RadiansPerSecond;
import static edu.wpi.first.units.Units.Volts;

import org.littletonrobotics.junction.Logger;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.MotionMagicExpoTorqueCurrentFOC;
import com.ctre.phoenix6.controls.TorqueCurrentFOC;
import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.GravityTypeValue;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import com.ctre.phoenix6.signals.StaticFeedforwardSignValue;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.filter.Debouncer;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Current;
import edu.wpi.first.units.measure.Temperature;
import edu.wpi.first.units.measure.Voltage;
import frc.robot.constants.pivot.PivotConfigBase;
import frc.robot.lib.util.PhoenixUtil;

public class PivotIOTalonFX implements PivotIO {
    private final PivotConfigBase config;

    private final TalonFX pivotMotor;
    private TalonFXConfiguration pivotConfig;

    private final StatusSignal<Angle> pivotPositionStatusSignal;
    private final StatusSignal<AngularVelocity> pivotVelocityStatusSignal;

    private final StatusSignal<Temperature> pivotTemperatureStatusSignal;

    private final StatusSignal<Voltage> pivotAppliedVolts;
    private final StatusSignal<Current> pivotTorqueCurrent;

    private final StatusSignal<Double> pivotClosedLoopReference;
    private final StatusSignal<Double> pivotClosedLoopOutput;

    private final Debouncer pivotConnectedDebouncer = new Debouncer(0.25, Debouncer.DebounceType.kBoth);

    private final MotionMagicExpoTorqueCurrentFOC pivotPositionRequest = 
        new MotionMagicExpoTorqueCurrentFOC(0).withSlot(0);
    private final TorqueCurrentFOC pivotTorqueRequest = new TorqueCurrentFOC(0);
    private final VoltageOut pivotVoltageRequest = new VoltageOut(0).withEnableFOC(false);

    private final double kMAX_ANGLE_ROTATIONS;
    private final double kMIN_ANGLE_ROTATIONS;
    private final double kSTARTING_ANGLE_RAD;

    public PivotIOTalonFX(PivotConfigBase config) {
        this.config = config;
        kMAX_ANGLE_ROTATIONS = config.getMaxAngleRotations();
        kMIN_ANGLE_ROTATIONS = config.getMinAngleRotations();
        kSTARTING_ANGLE_RAD = config.getStartingAngleRotations() * Math.PI * 2;

        pivotMotor = new TalonFX(config.getCanID());
        PhoenixUtil.tryUntilOk(5, () -> pivotMotor.getConfigurator().apply(pivotConfig, 0.25));

        this.pivotConfig = new TalonFXConfiguration();
        pivotConfig.Slot0.kP = config.getKP();
        pivotConfig.Slot0.kI = config.getKI();
        pivotConfig.Slot0.kD = config.getKD();
        pivotConfig.Slot0.kS = config.getKS();
        pivotConfig.Slot0.kV = config.getKV();
        pivotConfig.Slot0.kA = config.getKA();
        pivotConfig.Slot0.kG = config.getKG();

        pivotConfig.Slot0.GravityType = GravityTypeValue.Arm_Cosine;
        pivotConfig.Slot0.StaticFeedforwardSign = StaticFeedforwardSignValue.UseVelocitySign;

        pivotConfig.MotionMagic.MotionMagicExpo_kA = config.getMotionMagicExpoKA();
        pivotConfig.MotionMagic.MotionMagicExpo_kV = config.getMotionMagicExpoKV();
        pivotConfig.MotionMagic.MotionMagicCruiseVelocity = config.getMotionMagicCruiseVelocityRotationsPerSec();

        pivotConfig.ClosedLoopGeneral.ContinuousWrap = false;
        pivotConfig.Feedback.SensorToMechanismRatio = config.getMotorToOutputShaftRatio();

        pivotConfig.CurrentLimits.SupplyCurrentLimitEnable = true;
        pivotConfig.CurrentLimits.SupplyCurrentLimit = config.getSupplyCurrentLimit();
        pivotConfig.CurrentLimits.SupplyCurrentLowerLimit = config.getSupplyCurrentLimitLowerLimit();
        pivotConfig.CurrentLimits.SupplyCurrentLowerTime = config.getSupplyCurrentLimitLowerTime();

        pivotConfig.TorqueCurrent.PeakForwardTorqueCurrent = config.getPeakForwardTorqueCurrent();
        pivotConfig.TorqueCurrent.PeakReverseTorqueCurrent = config.getPeakReverseTorqueCurrent();

        pivotConfig.MotorOutput.NeutralMode = config.isNeutralModeBrake() ?
            NeutralModeValue.Brake :
            NeutralModeValue.Coast;
        pivotConfig.MotorOutput.Inverted = config.isInverted() ? 
            InvertedValue.Clockwise_Positive :
            InvertedValue.CounterClockwise_Positive;

        pivotConfig.FutureProofConfigs = true;

        pivotPositionStatusSignal = pivotMotor.getPosition().clone();
        pivotVelocityStatusSignal = pivotMotor.getVelocity().clone();

        pivotAppliedVolts = pivotMotor.getMotorVoltage().clone();
        pivotTorqueCurrent = pivotMotor.getStatorCurrent().clone();

        pivotTemperatureStatusSignal = pivotMotor.getDeviceTemp().clone();

        pivotClosedLoopReference = pivotMotor.getClosedLoopReference().clone();
        pivotClosedLoopOutput = pivotMotor.getClosedLoopOutput().clone();

        BaseStatusSignal.setUpdateFrequencyForAll(
            70,
            pivotAppliedVolts,
            pivotTorqueCurrent,
            pivotPositionStatusSignal,
            pivotVelocityStatusSignal,
            pivotClosedLoopReference,
            pivotClosedLoopOutput,
            pivotTemperatureStatusSignal
        );

        pivotMotor.optimizeBusUtilization();
    }

    @Override
    public void updateInputs(PivotIOInputs inputs) {
        inputs.pivotMotorConnected = 
            pivotConnectedDebouncer.calculate(
                BaseStatusSignal.refreshAll(
                    pivotAppliedVolts,
                    pivotTorqueCurrent,
                    pivotPositionStatusSignal,
                    pivotVelocityStatusSignal,
                    pivotClosedLoopReference,
                    pivotClosedLoopOutput,
                    pivotTemperatureStatusSignal
                ).isOK()
            );
        
        inputs.pivotPosition = new Rotation2d(
            BaseStatusSignal.getLatencyCompensatedValue(pivotPositionStatusSignal, pivotVelocityStatusSignal).in(Radians) +
            kSTARTING_ANGLE_RAD
        );

        inputs.pivotVelocityRadPerSec = pivotVelocityStatusSignal.getValue().in(RadiansPerSecond);

        inputs.pivotAppliedVolts = pivotAppliedVolts.getValue().in(Volts);
        inputs.pivotCurrentDrawAmps = pivotTorqueCurrent.getValue().in(Amps);

        inputs.pivotTemperature = pivotTemperatureStatusSignal.getValue().in(Fahrenheit);

        Logger.recordOutput("PivotComp/pivotClosedLoopReference", 2 * Math.PI * pivotClosedLoopReference.getValueAsDouble() + kSTARTING_ANGLE_RAD); //print loop reference (the value the pid targets)
        Logger.recordOutput("PivotComp/pivotClosedLoopOutput", pivotClosedLoopOutput.getValueAsDouble()); //print loop output
    }

    @Override
    public void setAngle(Rotation2d desiredPosition) {
        double positionRotations = desiredPosition.getRotations();

        pivotMotor.getConfigurator().apply(pivotConfig.MotionMagic.withMotionMagicCruiseVelocity(3));
        pivotMotor.setControl(
            pivotPositionRequest.withPosition(
                MathUtil.clamp(
                    positionRotations,
                    config.getMinAngleRotations(),
                    config.getMaxAngleRotations()
                )
            )
        );
    }

    @Override
    public void setTorqueCurrentFOC(double baseUnitMagnitude) {
        pivotMotor.setControl(
            pivotTorqueRequest.withOutput(baseUnitMagnitude)
        );
    }

    @Override
    public void setVoltage(double baseUnitMagnitude) {
        pivotMotor.setControl(
            pivotVoltageRequest.withOutput(baseUnitMagnitude)
        );
    }
}