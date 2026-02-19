package frc.robot.Shooter;

import static edu.wpi.first.units.Units.Degrees;
import static edu.wpi.first.units.Units.Inches;
import static edu.wpi.first.units.Units.Meters;
import static edu.wpi.first.units.Units.MetersPerSecond;
import static edu.wpi.first.units.Units.Rotations;
import com.ctre.phoenix6.CANBus;
import com.ctre.phoenix6.configs.MotionMagicConfigs;
import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.configs.SoftwareLimitSwitchConfigs;
import com.ctre.phoenix6.signals.StaticFeedforwardSignValue;

import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.units.measure.LinearVelocity;

public class Constants {
    public static final CANBus bus = new CANBus("rio");
    public static final Transform3d ShooterPlace = new Transform3d(0, 0, 0, Rotation3d.kZero);

   public class Pitch {
        public static final int MotorID = 54;
        public static final int EncoderID = 56;
        public static final double GearRatio = 10;
        public static final Angle MaxPitch = Degrees.of(55);
        public static final Angle PitchOffset = Rotations.of(0);
        public static final Slot0Configs PitchPID = new Slot0Configs()
            .withKP(0.1).withKD(0).withStaticFeedforwardSign(StaticFeedforwardSignValue.UseClosedLoopSign);
        public static final MotionMagicConfigs PitchMagic = new MotionMagicConfigs()
            .withMotionMagicCruiseVelocity(1)
            .withMotionMagicAcceleration(1);
        public static final SoftwareLimitSwitchConfigs PitchLimit = new SoftwareLimitSwitchConfigs()
            .withForwardSoftLimitEnable(true).withReverseSoftLimitEnable(true)
            .withForwardSoftLimitThreshold(MaxPitch).withReverseSoftLimitThreshold(0);

            
   }

   public class Shoot {
    public static final int MotorID = 55;
    public static final double GearRatio = 1;
    public static final Distance WheelCirc = Inches.of(4).times(Math.PI);
    public static final LinearVelocity MaxVelocity = MetersPerSecond.of(122.8/GearRatio*WheelCirc.in(Meters));
    public static final Slot0Configs ShootPID = new Slot0Configs()
        .withKP(0.1).withKD(0).withKV(12.4/122.8*GearRatio).withStaticFeedforwardSign(StaticFeedforwardSignValue.UseClosedLoopSign);
    public static final MotionMagicConfigs ShootMagic = new MotionMagicConfigs()
        .withMotionMagicCruiseVelocity(1)
        .withMotionMagicAcceleration(1);
   }

   public class Index {
    public static final int MotorID = 56;
    public static final int GearRatio = 1;
    public static Slot0Configs IndexPID = new Slot0Configs()
        .withKP(0).withKD(0).withKV(12.4/122.8*GearRatio) .withStaticFeedforwardSign(StaticFeedforwardSignValue.UseVelocitySign);
    public static MotionMagicConfigs IndexMagic = new MotionMagicConfigs()
        .withMotionMagicAcceleration(1);
   }

   public class Spindex {
    public static final int MotorID = 57;
    public static final int GearRatio = 1;
    public static final Slot0Configs SpindexPID = new Slot0Configs()
        .withKP(0).withKD(0).withKV(12.4/122.8*GearRatio); //kraken x44 with foc control
    public static final MotionMagicConfigs SpindexMagic = new MotionMagicConfigs()
        .withMotionMagicAcceleration(1);
   }
}
