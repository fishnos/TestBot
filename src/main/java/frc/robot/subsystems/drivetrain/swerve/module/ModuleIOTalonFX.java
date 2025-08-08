package frc.robot.subsystems.drivetrain.swerve.module;

import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import com.ctre.phoenix6.signals.StaticFeedforwardSignValue;

import edu.wpi.first.math.filter.Debouncer;
import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.units.measure.Current;
import edu.wpi.first.units.measure.Temperature;
import edu.wpi.first.wpilibj.motorcontrol.Talon;
import frc.robot.constants.drivetrain.swerve.module.SwerveModuleGeneralConfigBase;
import frc.robot.constants.drivetrain.swerve.module.SwerveModuleSpecificConfigBase;
import frc.robot.lib.util.PhoenixUtil;

public class ModuleIOTalonFX {
    private final SwerveModuleGeneralConfigBase generalConfig;
    private final SwerveModuleSpecificConfigBase specificConfig;

    private final TalonFXConfiguration driveConfig;
    private final TalonFX driveMotor;

    private final StatusSignal<Temperature> driveTemp;
    private StatusSignal<Current> driveTorqueCurrent;

    private final TalonFXConfiguration steerConfig;
    private final TalonFX steerMotor;

    private final SlewRateLimiter driveAccelLimiter;

    private final Debouncer driveConnectedDebouncer = new Debouncer(0.25, Debouncer.DebounceType.kBoth);
    private final Debouncer steerConnectedDebouncer = new Debouncer(0.25, Debouncer.DebounceType.kBoth);
    private final Debouncer steerEncoderConnectedDebouncer = new Debouncer(0.25, Debouncer.DebounceType.kBoth);

    public ModuleIOTalonFX(SwerveModuleGeneralConfigBase generalConfig, SwerveModuleSpecificConfigBase specificConfig, int moduleID) {
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
            generalConfig.getDriveWheelRadiusMeters() * 2 * Math.PI;
        
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

        steerConfig = new TalonFXConfiguration();

        steerMotor = new TalonFX(specificConfig.getSteerCanId(), generalConfig.getCanBusName());

        driveTemp = driveMotor.getDeviceTemp().clone();

        driveMotor.optimizeBusUtilization();
    }
}
