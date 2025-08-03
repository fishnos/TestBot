package frc.robot.subsystems.drivetrain.swerve.gyro;

import static edu.wpi.first.units.Units.Fahrenheit;
import static edu.wpi.first.units.Units.MetersPerSecond;
import static edu.wpi.first.units.Units.MetersPerSecondPerSecond;
import static edu.wpi.first.units.Units.Radians;
import static edu.wpi.first.units.Units.RadiansPerSecond;

import java.io.ObjectInputFilter.Status;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.Pigeon2Configuration;
import com.ctre.phoenix6.hardware.Pigeon2;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.filter.Debouncer;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.LinearAcceleration;
import edu.wpi.first.units.measure.Temperature;
import frc.robot.lib.util.PhoenixUtil;
import frc.robot.subsystems.drivetrain.swerve.Phoenix6Odometry;

public class GyroIOPigeon2 implements GyroIO {
    private final Pigeon2Configuration pigeon2Config;
    private final Pigeon2 pigeon2;

    private final StatusSignal<Angle> gyroYawStatusSignal;
    private final StatusSignal<Angle> gyroPitchStatusSignal;
    private final StatusSignal<Angle> gyroRollStatusSignal;

    private final StatusSignal<LinearAcceleration> gyroAccelXStatusSignal;
    private final StatusSignal<LinearAcceleration> gyroAccelYStatusSignal;
    private final StatusSignal<LinearAcceleration> gyroAccelZStatusSignal;

    private final StatusSignal<AngularVelocity> gyroYawVelocityStatusSignal;
    private final StatusSignal<AngularVelocity> gyroPitchVelocityStatusSignal;
    private final StatusSignal<AngularVelocity> gyroRollVelocityStatusSignal;

    private final StatusSignal<Temperature> gyroTemperatureStatusSignal;

    private final Phoenix6Odometry odom;

    private final Debouncer connectedDebouncer = new Debouncer(0.25, Debouncer.DebounceType.kBoth);

    public GyroIOPigeon2() {
        odom = Phoenix6Odometry.getInstance();

        pigeon2 = new Pigeon2(12, "drivetrain");
        pigeon2Config = new Pigeon2Configuration();

        gyroYawStatusSignal = pigeon2.getYaw().clone();
        gyroPitchStatusSignal = pigeon2.getPitch().clone();
        gyroRollStatusSignal = pigeon2.getRoll().clone();

        gyroAccelXStatusSignal = pigeon2.getAccelerationX().clone();
        gyroAccelYStatusSignal = pigeon2.getAccelerationY().clone();
        gyroAccelZStatusSignal = pigeon2.getAccelerationZ().clone();

        gyroRollVelocityStatusSignal = pigeon2.getAngularVelocityXWorld().clone();
        gyroPitchVelocityStatusSignal = pigeon2.getAngularVelocityYWorld().clone();
        gyroYawVelocityStatusSignal = pigeon2.getAngularVelocityZWorld().clone();

        gyroTemperatureStatusSignal = pigeon2.getTemperature().clone();

        PhoenixUtil.tryUntilOk(5, () -> pigeon2.getConfigurator().apply(pigeon2Config, 0.25));

        BaseStatusSignal.setUpdateFrequencyForAll(
            70,
            gyroYawStatusSignal,
            gyroPitchStatusSignal,
            gyroRollStatusSignal,
            gyroAccelXStatusSignal,
            gyroAccelYStatusSignal,
            gyroAccelZStatusSignal,
            gyroYawVelocityStatusSignal,
            gyroPitchVelocityStatusSignal,
            gyroRollVelocityStatusSignal,
            gyroTemperatureStatusSignal
        );

        odom.registerSignal(pigeon2, gyroYawStatusSignal);
        odom.registerSignal(pigeon2, gyroPitchStatusSignal);
        odom.registerSignal(pigeon2, gyroRollStatusSignal);
        odom.registerSignal(pigeon2, gyroAccelXStatusSignal);
        odom.registerSignal(pigeon2, gyroAccelYStatusSignal);
        odom.registerSignal(pigeon2, gyroAccelZStatusSignal);
        odom.registerSignal(pigeon2, gyroYawVelocityStatusSignal);
        odom.registerSignal(pigeon2, gyroPitchVelocityStatusSignal);
        odom.registerSignal(pigeon2, gyroRollVelocityStatusSignal);
        odom.registerSignal(pigeon2, gyroTemperatureStatusSignal);

        pigeon2.optimizeBusUtilization();
    }

    @Override
    public void updateInputs(GyroIOInputs inputs) {
        inputs.isConnected = connectedDebouncer.calculate(
            BaseStatusSignal.refreshAll(
                gyroYawStatusSignal,
                gyroPitchStatusSignal,
                gyroRollStatusSignal,
                gyroAccelXStatusSignal,
                gyroAccelYStatusSignal,
                gyroAccelZStatusSignal,
                gyroYawVelocityStatusSignal,
                gyroPitchVelocityStatusSignal,
                gyroRollVelocityStatusSignal,
                gyroTemperatureStatusSignal
            ).isOK()
        );

        inputs.gyroTemperature = gyroTemperatureStatusSignal.getValue().in(Fahrenheit);

        inputs.orientation = new Rotation2d[] {
            new Rotation2d(MathUtil.angleModulus(BaseStatusSignal.getLatencyCompensatedValue(gyroRollStatusSignal, gyroRollVelocityStatusSignal).in(Radians))),
            new Rotation2d(MathUtil.angleModulus(BaseStatusSignal.getLatencyCompensatedValue(gyroPitchStatusSignal, gyroPitchVelocityStatusSignal).in(Radians))),
            new Rotation2d(MathUtil.angleModulus(BaseStatusSignal.getLatencyCompensatedValue(gyroYawStatusSignal, gyroYawVelocityStatusSignal).in(Radians))),
        };

        //doesnt need latency compensation due to the fact that velocity is a rate, not position (also negligible latency)
        inputs.rates = new Rotation2d[] {
            new Rotation2d(gyroRollVelocityStatusSignal.getValue().in(RadiansPerSecond)),
            new Rotation2d(gyroPitchVelocityStatusSignal.getValue().in(RadiansPerSecond)),
            new Rotation2d(gyroYawVelocityStatusSignal.getValue().in(RadiansPerSecond))
        };

        //need a jerk signal for acceleration latency compensation, since jerk is the derivative of acceleration (dont have one though)
        inputs.fieldRelativeAccelMetersPerSec = new Translation2d(
            gyroAccelXStatusSignal.getValue().in(MetersPerSecondPerSecond),
            gyroAccelYStatusSignal.getValue().in(MetersPerSecondPerSecond)
        );
    }

    @Override
    public void resetGyro(Rotation2d yaw) {
        pigeon2.setYaw(yaw.getDegrees());
    }
}
