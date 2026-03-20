package frc.robot.Shooter;

import static edu.wpi.first.units.Units.Degrees;
import static edu.wpi.first.units.Units.Inches;
import static edu.wpi.first.units.Units.MetersPerSecond;
import static edu.wpi.first.units.Units.Millimeters;
import static edu.wpi.first.units.Units.Rotations;
import com.ctre.phoenix6.CANBus;
import com.ctre.phoenix6.configs.MotionMagicConfigs;
import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.configs.SoftwareLimitSwitchConfigs;
import com.ctre.phoenix6.signals.StaticFeedforwardSignValue;

import edu.wpi.first.math.Pair;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.units.measure.LinearVelocity;

public class Constants {
    public static final CANBus bus = new CANBus("rio");
    public static final Transform3d ShooterPlace = new Transform3d(Millimeters.of(89.20000), Millimeters.of(74.80000), Millimeters.of(402.60000), new Rotation3d(Degrees.of(0), Degrees.of(0),Degrees.of(0)));

   public class Pitch {

        public static final int MotorID = 54;
        public static final int EncoderID = 53;
        public static final double GearRatio = 20;
        public static final Pair<Angle, Angle> PitchingAngle = Pair.of(Degrees.of(26), Degrees.of(38));
        public static final Pair<Angle, Angle> PitchConstraints = Pair.of(Degrees.of(90).minus(PitchingAngle.getSecond()),Degrees.of(90).minus(PitchingAngle.getFirst()));
        public static final Angle PitchOffset = Rotations.of(0);// Degrees.of(256.72407)
        public static final Slot0Configs PitchPID = new Slot0Configs() //PositionVoltage
            .withKP(3.55).withKD(0)
            .withKS(0.44).withKV(0).withStaticFeedforwardSign(StaticFeedforwardSignValue.UseClosedLoopSign);
        public static final MotionMagicConfigs PitchMagic = new MotionMagicConfigs()
            .withMotionMagicCruiseVelocity(0.06)
            .withMotionMagicAcceleration(0.75);
        public static final SoftwareLimitSwitchConfigs PitchLimit = new SoftwareLimitSwitchConfigs()
            .withForwardSoftLimitEnable(true).withReverseSoftLimitEnable(true)
            .withReverseSoftLimitThreshold(PitchingAngle.getFirst()).withForwardSoftLimitThreshold(PitchingAngle.getSecond());
            
   }

   public class Shoot {
    public static final int MotorID = 55;
    public static final double GearRatio = 1;
    public static final Distance WheelRadius = Inches.of(2);
    public static final LinearVelocity MaxVelocity = MetersPerSecond.of(36);
    
    // TorqueCurrentFOC control set
    public static final Slot0Configs ShootPID = new Slot0Configs()
        .withKP(6).withKS(2.5).withStaticFeedforwardSign(StaticFeedforwardSignValue.UseVelocitySign);
    

    // velocity control set
    // public static final Slot0Configs ShootPID = new Slot0Configs()
    //     .withKP(0.35).withKD(0.003)
    //     .withKS(0.32).withKV(0.07)
    //     .withStaticFeedforwardSign(StaticFeedforwardSignValue.UseVelocitySign);
    public static final MotionMagicConfigs ShootMagic = new MotionMagicConfigs()
        .withMotionMagicCruiseVelocity(5)
        .withMotionMagicAcceleration(60);
   }

   
}
