package frc.robot.subsystems.drivetrain.swerve.module;

import static edu.wpi.first.units.Units.Amps;
import static edu.wpi.first.units.Units.Fahrenheit;
import static edu.wpi.first.units.Units.Radians;
import static edu.wpi.first.units.Units.RadiansPerSecond;
import static edu.wpi.first.units.Units.Rotations;
import static edu.wpi.first.units.Units.RotationsPerSecond;
import static edu.wpi.first.units.Units.Volts;

import java.util.Optional;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.CANcoderConfiguration;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.MotionMagicTorqueCurrentFOC;
import com.ctre.phoenix6.controls.VelocityTorqueCurrentFOC;
import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import com.ctre.phoenix6.signals.StaticFeedforwardSignValue;

import edu.wpi.first.hal.HALUtil;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.filter.Debouncer;
import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularAcceleration;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Current;
import edu.wpi.first.units.measure.Temperature;
import edu.wpi.first.units.measure.Voltage;
import frc.robot.constants.drivetrain.swerve.module.SwerveModuleGeneralConfigBase;
import frc.robot.constants.drivetrain.swerve.module.SwerveModuleSpecificConfigBase;
import frc.robot.lib.util.PhoenixUtil;

public class ModuleIOTalonFX implements ModuleIO {
    private final SwerveModuleGeneralConfigBase generalConfig;
    private final SwerveModuleSpecificConfigBase specificConfig;

    private final int moduleID;

    private final TalonFXConfiguration driveConfig;
    private final TalonFX driveMotor;

    private final SlewRateLimiter driveAccelLimiter;

    private final StatusSignal<Angle> drivePosition;
    private final StatusSignal<AngularVelocity> driveVelocity;
    private final StatusSignal<AngularAcceleration> driveAcceleration;

    private final StatusSignal<Current> driveTorqueCurrent;
    private final StatusSignal<Voltage> driveAppliedVoltage;
    private final StatusSignal<Temperature> driveTemp;

    private final TalonFXConfiguration steerConfig;
    private final CANcoderConfiguration encoderConfig;

    private final CANcoder steerEncoder;
    private final TalonFX steerMotor;

    private final StatusSignal<Angle> steerMotorPosition;
    private final StatusSignal<AngularVelocity> steerMotorVelocity;
    private final StatusSignal<AngularAcceleration> steerMotorAcceleration;
    private final StatusSignal<Current> steerMotorTorqueCurrent;
    private final StatusSignal<Voltage> steerMotorAppliedVoltage;
    private final StatusSignal<Temperature> steerMotorTemperature;

    private final StatusSignal<Angle> steerEncoderPosition;
    private final StatusSignal<Angle> steerEncoderAbsolutePosition;
    private final StatusSignal<AngularVelocity> steerEncoderVelocity;

    private final VelocityTorqueCurrentFOC driveVelocityRequest = new VelocityTorqueCurrentFOC(0);
    private final MotionMagicTorqueCurrentFOC steerMotorRequest = new MotionMagicTorqueCurrentFOC(0);

    private final Debouncer driveConnectedDebouncer = new Debouncer(0.25, Debouncer.DebounceType.kBoth);
    private final Debouncer steerConnectedDebouncer = new Debouncer(0.25, Debouncer.DebounceType.kBoth);
    private final Debouncer steerEncoderConnectedDebouncer = new Debouncer(0.25, Debouncer.DebounceType.kBoth);

    private double currentDriveVelocity = 0;

    private double lastSteerPosition = 0;
    private Rotation2d lastSteerSetpoint = new Rotation2d();

    public ModuleIOTalonFX(SwerveModuleGeneralConfigBase generalConfig, SwerveModuleSpecificConfigBase specificConfig, int moduleID) {
        this.moduleID = moduleID;

        this.generalConfig = generalConfig;
        this.specificConfig = specificConfig;

        driveConfig = new TalonFXConfiguration();

        driveConfig.Slot0.kP = generalConfig.getDriveKP();
        driveConfig.Slot0.kI = generalConfig.getDriveKI();
        driveConfig.Slot0.kD = generalConfig.getDriveKD();
        driveConfig.Slot0.kA = generalConfig.getDriveKA();
        driveConfig.Slot0.kV = generalConfig.getDriveKV();
        driveConfig.Slot0.kS = generalConfig.getDriveKS();
        driveConfig.Slot0.StaticFeedforwardSign = StaticFeedforwardSignValue.UseVelocitySign;

        driveConfig.MotionMagic.MotionMagicAcceleration = generalConfig.getDriveMotionMagicVelocityAccelerationMetersPerSecSec();
        driveConfig.MotionMagic.MotionMagicJerk = generalConfig.getDriveMotionMagicVelocityJerkMetersPerSecSecSec();
        driveAccelLimiter = new SlewRateLimiter(generalConfig.getDriveMotionMagicVelocityJerkMetersPerSecSecSec());

        driveConfig.CurrentLimits.SupplyCurrentLimitEnable = true;
        driveConfig.CurrentLimits.SupplyCurrentLimit = generalConfig.getDriveSupplyCurrentLimit();
        driveConfig.CurrentLimits.SupplyCurrentLowerLimit = generalConfig.getDriveSupplyCurrentLimitLowerLimit();
        driveConfig.CurrentLimits.SupplyCurrentLowerTime = generalConfig.getDriveSupplyCurrentLimitLowerTime();

        driveConfig.CurrentLimits.StatorCurrentLimitEnable = true;
        driveConfig.CurrentLimits.StatorCurrentLimit = generalConfig.getDriveStatorCurrentLimit();

        driveConfig.TorqueCurrent.PeakForwardTorqueCurrent = generalConfig.getDrivePeakForwardTorqueCurrent();
        driveConfig.TorqueCurrent.PeakReverseTorqueCurrent = generalConfig.getDrivePeakReverseTorqueCurrent();

        driveConfig.ClosedLoopGeneral.ContinuousWrap = false;
        driveConfig.Feedback.SensorToMechanismRatio = 
            generalConfig.getDriveMotorToOutputShaftRatio() / 
            (generalConfig.getDriveWheelRadiusMeters() * 2 * Math.PI);
        
        driveConfig.MotorOutput.NeutralMode = 
            generalConfig.getIsDriveNeutralModeBrake() ? 
            NeutralModeValue.Brake : 
            NeutralModeValue.Coast;
        
        driveConfig.MotorOutput.Inverted =
            specificConfig.getIsDriveInverted() ?
            InvertedValue.Clockwise_Positive :
            InvertedValue.CounterClockwise_Positive;

        driveConfig.FutureProofConfigs = true;

        driveMotor = new TalonFX(specificConfig.getDriveCanId(), generalConfig.getCanBusName());
        PhoenixUtil.tryUntilOk(5, () -> driveMotor.getConfigurator().apply(driveConfig, 0.25));

        encoderConfig = new CANcoderConfiguration();

        encoderConfig.MagnetSensor.withAbsoluteSensorDiscontinuityPoint(generalConfig.getCancoderAbsoluteSensorDiscontinuityPoint())
        .withMagnetOffset(specificConfig.getCancoderOffsetRotations())
        .withSensorDirection(generalConfig.getCancoderSensorDirection());

        encoderConfig.FutureProofConfigs = true;

        steerEncoder = new CANcoder(specificConfig.getCancoderCanId(), generalConfig.getCanBusName());
        PhoenixUtil.tryUntilOk(5, () -> steerEncoder.getConfigurator().apply(encoderConfig, 0.25));

        steerConfig = new TalonFXConfiguration();

        steerConfig.Slot0.kP = generalConfig.getSteerKP();
        steerConfig.Slot0.kI = generalConfig.getSteerKI();
        steerConfig.Slot0.kD = generalConfig.getSteerKD();
        steerConfig.Slot0.kA = generalConfig.getSteerKA();
        steerConfig.Slot0.kV = generalConfig.getSteerKV();
        steerConfig.Slot0.kS = generalConfig.getSteerKS();
        steerConfig.Slot0.StaticFeedforwardSign = StaticFeedforwardSignValue.UseVelocitySign;

        steerConfig.MotionMagic.MotionMagicExpo_kA = generalConfig.getSteerMotionMagicExpoKA();
        steerConfig.MotionMagic.MotionMagicExpo_kV = generalConfig.getSteerMotionMagicExpoKV();
        //limit the velo for steer because of precision, do not for drive because accel needs to be limited but velo needs to be hit as fast as possible
        steerConfig.MotionMagic.MotionMagicCruiseVelocity = generalConfig.getSteerMotionMagicCruiseVelocityRotationsPerSec();

        steerConfig.CurrentLimits.SupplyCurrentLimitEnable = true;
        steerConfig.CurrentLimits.SupplyCurrentLimit = generalConfig.getSteerSupplyCurrentLimit();
        steerConfig.CurrentLimits.SupplyCurrentLowerLimit = generalConfig.getSteerSupplyCurrentLimitLowerLimit();
        steerConfig.CurrentLimits.SupplyCurrentLowerTime = generalConfig.getSteerSupplyCurrentLimitLowerTime();

        steerConfig.CurrentLimits.StatorCurrentLimitEnable = true;
        steerConfig.CurrentLimits.StatorCurrentLimit = generalConfig.getSteerStatorCurrentLimit();

        steerConfig.TorqueCurrent.PeakForwardTorqueCurrent = generalConfig.getSteerPeakForwardTorqueCurrent();
        steerConfig.TorqueCurrent.PeakReverseTorqueCurrent = generalConfig.getSteerPeakReverseTorqueCurrent();

        steerConfig.MotorOutput.NeutralMode = 
            generalConfig.getIsSteerNeutralModeBrake() ? 
            NeutralModeValue.Brake : 
            NeutralModeValue.Coast;
        
        steerConfig.MotorOutput.Inverted =
            specificConfig.getIsSteerInverted() ?
            InvertedValue.Clockwise_Positive :
            InvertedValue.CounterClockwise_Positive;

        steerConfig.ClosedLoopGeneral.ContinuousWrap = true;
        steerConfig.Feedback.SensorToMechanismRatio = 1; //use the cancoder position directly, no ratio needed
        steerConfig.Feedback.FeedbackRemoteSensorID = specificConfig.getCancoderCanId(); //use the cancoder for remote feedback
        steerConfig.Feedback.FeedbackSensorSource = generalConfig.getSteerCancoderFeedbackSensorSource(); //fuse cancoder position with motor position
        steerConfig.Feedback.RotorToSensorRatio = generalConfig.getSteerRotorToSensorRatio(); //how many times the motor turns for one rotation

        steerConfig.FutureProofConfigs = true;

        steerMotor = new TalonFX(specificConfig.getSteerCanId(), generalConfig.getCanBusName());
        PhoenixUtil.tryUntilOk(5, () -> steerMotor.getConfigurator().apply(steerConfig, 0.25));

        drivePosition = driveMotor.getPosition().clone();
        driveVelocity = driveMotor.getVelocity().clone();
        driveAcceleration = driveMotor.getAcceleration().clone();

        driveTorqueCurrent = driveMotor.getTorqueCurrent().clone();
        driveAppliedVoltage = driveMotor.getMotorVoltage().clone();
        driveTemp = driveMotor.getDeviceTemp().clone();

        steerMotorPosition = steerMotor.getPosition().clone();
        steerMotorVelocity = steerMotor.getVelocity().clone();
        steerMotorAcceleration = steerMotor.getAcceleration().clone();

        steerMotorTorqueCurrent = steerMotor.getTorqueCurrent().clone();
        steerMotorAppliedVoltage = steerMotor.getMotorVoltage().clone();
        steerMotorTemperature = steerMotor.getDeviceTemp().clone();

        steerEncoderPosition = steerEncoder.getPosition().clone();
        steerEncoderAbsolutePosition = steerEncoder.getAbsolutePosition().clone();
        steerEncoderVelocity = steerEncoder.getVelocity().clone();

        BaseStatusSignal.setUpdateFrequencyForAll(
            100, 
            drivePosition,
            driveVelocity,
            driveAcceleration,
            driveTorqueCurrent,
            driveAppliedVoltage,
            driveTemp
        );

        BaseStatusSignal.setUpdateFrequencyForAll(
            250,
            steerMotorPosition,
            steerMotorVelocity,
            steerMotorAcceleration,
            steerMotorAppliedVoltage,
            steerMotorTorqueCurrent,
            steerMotorTemperature,

            steerEncoderAbsolutePosition,
            steerEncoderPosition,
            steerEncoderVelocity
        );

        driveMotor.optimizeBusUtilization();
        steerMotor.optimizeBusUtilization();
        steerEncoder.optimizeBusUtilization();
    }

    @Override
    public void updateInputs(ModuleIOInputs inputs) {
        BaseStatusSignal.refreshAll(
            drivePosition,
            driveVelocity,
            driveAcceleration,
            driveTorqueCurrent,
            driveAppliedVoltage,
            driveTemp,

            steerMotorPosition,
            steerMotorVelocity,
            steerMotorAcceleration,
            steerMotorAppliedVoltage,
            steerMotorTorqueCurrent,
            steerMotorTemperature,

            steerEncoderAbsolutePosition,
            steerEncoderPosition,
            steerEncoderVelocity
        );

        inputs.driveConnected = 
            driveConnectedDebouncer.calculate(
                BaseStatusSignal.refreshAll(
                    driveAcceleration,
                    driveTorqueCurrent,
                    driveAppliedVoltage,
                    driveTemp
                ).isOK() && BaseStatusSignal.isAllGood(
                    drivePosition,
                    driveVelocity
                )
            );
        
        inputs.steerConnected =
            steerConnectedDebouncer.calculate(
                BaseStatusSignal.refreshAll(
                    steerMotorAcceleration,
                    steerMotorAppliedVoltage,
                    steerMotorTorqueCurrent,
                    steerMotorTemperature 
                ).isOK() && BaseStatusSignal.isAllGood(
                    steerMotorPosition,
                    steerMotorVelocity
                )
            );
        inputs.steerEncoderConnected =
            steerEncoderConnectedDebouncer.calculate(
                BaseStatusSignal.refreshAll(
                    steerEncoderAbsolutePosition,
                    steerEncoderPosition,
                    steerEncoderVelocity
                ).isOK()
            );

        inputs.driveAppliedVolts = driveAppliedVoltage.getValue().in(Volts);
        inputs.driveCurrentDrawAmps = driveTorqueCurrent.getValue().in(Amps);
        inputs.driveTemperature = driveTemp.getValue().in(Fahrenheit);

        inputs.drivePositionMeters = 
            BaseStatusSignal.getLatencyCompensatedValue(drivePosition, driveVelocity).in(Rotations);
        inputs.driveVelocityMetersPerSec =
            BaseStatusSignal.getLatencyCompensatedValue(driveVelocity, driveAcceleration).in(RotationsPerSecond);
        currentDriveVelocity = inputs.driveVelocityMetersPerSec;
        
        inputs.steerAppliedVolts = steerMotorAppliedVoltage.getValue().in(Volts);
        inputs.steerTemperature = steerMotorTemperature.getValue().in(Fahrenheit);
        inputs.steerCurrentDrawAmps = steerMotorTorqueCurrent.getValue().in(Amps);

        inputs.steerPositionRadians = 
            new Rotation2d(
                BaseStatusSignal.getLatencyCompensatedValue(steerMotorPosition, steerMotorVelocity).in(Radians)
            );
        lastSteerPosition = inputs.steerPositionRadians.getRadians();
        inputs.steerVelocityRadiansPerSec = 
            BaseStatusSignal.getLatencyCompensatedValue(steerMotorVelocity, steerMotorAcceleration).in(RadiansPerSecond);

        inputs.timestamp = HALUtil.getFPGATime() / 1e6; //seconds
    }

    @Override
    public void setState(SwerveModuleState state, Optional<Double> accelerationMetersPerSec) {
        double driveAccel = 
            driveAccelLimiter.calculate(
                accelerationMetersPerSec.isPresent() ?
                accelerationMetersPerSec.get().doubleValue() :
                0
            );
            
        double cosCompensantion = 
            Math.max(0.0, Math.cos(Math.abs(MathUtil.angleModulus(lastSteerSetpoint.getRadians() - lastSteerPosition))));
        
        driveMotor.setControl(
            driveVelocityRequest.withVelocity(
                MathUtil.clamp(
                    state.speedMetersPerSecond,
                    -generalConfig.getDriveMaxVelocityMetersPerSec(),
                    generalConfig.getDriveMaxVelocityMetersPerSec()
                ) * cosCompensantion
            ).withAcceleration(driveAccel)
        );

        steerMotor.setControl(
            steerMotorRequest.withPosition(
                state.angle.getRotations()
            )
        );

        lastSteerSetpoint = state.angle;
    }
}