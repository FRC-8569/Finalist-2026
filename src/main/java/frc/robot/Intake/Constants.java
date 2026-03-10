package frc.robot.Intake;

import static edu.wpi.first.units.Units.Amps;
import static edu.wpi.first.units.Units.Centimeters;
import static edu.wpi.first.units.Units.Inches;
import com.ctre.phoenix6.CANBus;
import com.ctre.phoenix6.configs.CurrentLimitsConfigs;
import com.ctre.phoenix6.configs.MotionMagicConfigs;
import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.configs.SoftwareLimitSwitchConfigs;
import com.ctre.phoenix6.signals.StaticFeedforwardSignValue;

import edu.wpi.first.units.measure.Current;
import edu.wpi.first.units.measure.Distance;

public class Constants {
    public static final CANBus bus = new CANBus("Drivetrain");
    public class Tongue {
        public static final int MotorID = 51;
        public static final Slot0Configs TonguePID = new Slot0Configs()
            .withKP(8).withKD(0.3)
            .withStaticFeedforwardSign(StaticFeedforwardSignValue.UseClosedLoopSign);
        
        public static final MotionMagicConfigs TongueMagic = new MotionMagicConfigs()
            .withMotionMagicExpo_kV(0.005)
            .withMotionMagicExpo_kA(0.25);

        public static final SoftwareLimitSwitchConfigs TongueLimit = new SoftwareLimitSwitchConfigs()
            .withForwardSoftLimitThreshold(1.45)
            .withReverseSoftLimitThreshold(0)
            .withForwardSoftLimitEnable(true)
            .withReverseSoftLimitEnable(true);

        public static final double GearRatio = 20*1/1.5;
        public static final Distance tolerance = Centimeters.of(1);
        public static final Current StallLimit = Amps.of(15); 
    }

    public class Roller {
        public static final int MotorID = 52;
        public static final Slot0Configs RollerPID = new Slot0Configs()
            .withKP(0.7).withKV(12.4/125.5*Roller.GearRatio)
            .withStaticFeedforwardSign(StaticFeedforwardSignValue.UseVelocitySign);
        public static final MotionMagicConfigs RollerMagic = new MotionMagicConfigs()
            .withMotionMagicAcceleration(100);
        public static final double GearRatio = 32.0/15;
        public static final Distance WheelRadius = Inches.of(2.25).div(2);
        public static final double[] SpeedRange = {5,10.5};
    }

}