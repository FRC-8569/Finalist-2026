package frc.robot.Climber;

import com.ctre.phoenix6.CANBus;
import com.ctre.phoenix6.configs.MotionMagicConfigs;
import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.configs.SoftwareLimitSwitchConfigs;

public class Constants {
    public static final int MotorID = 60;
    public static final CANBus bus = new CANBus("Drivetrain");
    public static final double GearRatio = 60;
    public static final Slot0Configs ClimbPID = new Slot0Configs()
        .withKP(0).withKS(0).withKD(0);
    public static final MotionMagicConfigs ClimbMagic = new MotionMagicConfigs()
        .withMotionMagicExpo_kV(0)
        .withMotionMagicExpo_kA(0);
    public static final SoftwareLimitSwitchConfigs ClimbLimit = new SoftwareLimitSwitchConfigs()
        .withForwardSoftLimitEnable(true)
        .withReverseSoftLimitEnable(true)
        .withForwardSoftLimitThreshold(240)
        .withReverseSoftLimitThreshold(0);
}
