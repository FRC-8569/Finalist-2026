package frc.robot.Drivetrain;

import static edu.wpi.first.units.Units.Amps;
import static edu.wpi.first.units.Units.Centimeters;
import static edu.wpi.first.units.Units.Inches;
import static edu.wpi.first.units.Units.KilogramSquareMeters;
import static edu.wpi.first.units.Units.Meters;
import static edu.wpi.first.units.Units.MetersPerSecond;
import static edu.wpi.first.units.Units.MetersPerSecondPerSecond;
import static edu.wpi.first.units.Units.Percent;
import static edu.wpi.first.units.Units.Rotations;
import static edu.wpi.first.units.Units.RotationsPerSecond;
import static edu.wpi.first.units.Units.RotationsPerSecondPerSecond;
import static edu.wpi.first.units.Units.Volts;

import com.ctre.phoenix6.CANBus;
import com.ctre.phoenix6.configs.CANcoderConfiguration;
import com.ctre.phoenix6.configs.CurrentLimitsConfigs;
import com.ctre.phoenix6.configs.MotionMagicConfigs;
import com.ctre.phoenix6.configs.Pigeon2Configuration;
import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.swerve.SwerveModuleConstants.SteerFeedbackType;
import com.ctre.phoenix6.signals.GainSchedBehaviorValue;
import com.ctre.phoenix6.signals.StaticFeedforwardSignValue;
import com.ctre.phoenix6.swerve.SwerveDrivetrainConstants;
import com.ctre.phoenix6.swerve.SwerveModuleConstants;
import com.ctre.phoenix6.swerve.SwerveModuleConstants.ClosedLoopOutputType;
import com.ctre.phoenix6.swerve.SwerveModuleConstants.DriveMotorArrangement;
import com.ctre.phoenix6.swerve.SwerveModuleConstants.SteerMotorArrangement;
import com.ctre.phoenix6.swerve.SwerveModuleConstantsFactory;
import com.pathplanner.lib.path.PathConstraints;

import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Current;
import edu.wpi.first.units.measure.Dimensionless;
import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.units.measure.LinearVelocity;
import edu.wpi.first.units.measure.MomentOfInertia;
import edu.wpi.first.units.measure.Voltage;

public class Constants {
    public static final Slot0Configs DrivePID = new Slot0Configs()
        .withKP(0.1).withKV(12.4/100*Constants.DriveGearRatio)
        .withStaticFeedforwardSign(StaticFeedforwardSignValue.UseClosedLoopSign)
        .withGainSchedBehavior(GainSchedBehaviorValue.UseSlot0);
    public static final Slot0Configs SteerPID = new Slot0Configs()
        .withKP(80).withKD(0)
        .withStaticFeedforwardSign(StaticFeedforwardSignValue.UseClosedLoopSign);
    public static final MotionMagicConfigs SteerMagic = new MotionMagicConfigs()
        .withMotionMagicExpo_kV(0.1)
        .withMotionMagicExpo_kA(0.06642857143); //0.6642857143

    public static final ClosedLoopOutputType DriveOutput = ClosedLoopOutputType.TorqueCurrentFOC;
    public static final ClosedLoopOutputType SteerOutput = ClosedLoopOutputType.TorqueCurrentFOC;
    public static final DriveMotorArrangement DriveMotor = DriveMotorArrangement.TalonFX_Integrated;
    public static final SteerMotorArrangement SteerMotor = SteerMotorArrangement.TalonFX_Integrated;
    public static final SteerFeedbackType SteerEncoder = SteerFeedbackType.FusedCANcoder;

    public static final Current SlipCurrent = Amps.of(40);
    public static final double DriveGearRatio = 1/(14.0/50*28/16*15/45); // TODO: SDS mk4i, formula implementation needed
    public static final double SteerGearRatio = 150.0/7;
    public static final double CoupleGearRatio = DriveGearRatio/SteerGearRatio;
    public static final Distance WheelRadius = Inches.of(2);
    public static final LinearVelocity MaxVelocity = MetersPerSecond.of(5);
    public static final LinearVelocity DriveVelocity = MetersPerSecond.of(4);
    public static final AngularVelocity MaxOmega = RotationsPerSecond.of(1.25);
    public static final Dimensionless Deadband = Percent.of(10); 
    public static final PathConstraints AutoConstraints = new PathConstraints(MetersPerSecond.of(5), MetersPerSecondPerSecond.of(8), MaxOmega, RotationsPerSecondPerSecond.of(20));

    public static final TalonFXConfiguration DriveConfig = new TalonFXConfiguration();
    public static final TalonFXConfiguration SteerConfig = new TalonFXConfiguration()
        .withCurrentLimits(
                new CurrentLimitsConfigs()
                    // Swerve azimuth does not require much torque output, so we can set a relatively low
                    // stator current limit to help avoid brownouts without impacting performance.
                    .withStatorCurrentLimit(Amps.of(60))
                    .withStatorCurrentLimitEnable(true)
            );
    public static final double[] RotationPIDConfig = {50/(2*Math.PI),0.0/(2*Math.PI),0.0/(2*Math.PI)};
    public static final CANcoderConfiguration EncoderConfig = new CANcoderConfiguration();
    public static final Pigeon2Configuration GyroConfig = null;
    public static final CANBus bus = new CANBus("Drivetrain");

    // These are only used for simulation
    private static final MomentOfInertia kSteerInertia = KilogramSquareMeters.of(0.01);
    private static final MomentOfInertia kDriveInertia = KilogramSquareMeters.of(0.01);
    // Simulated voltage necessary to overcome friction
    private static final Voltage kSteerFrictionVoltage = Volts.of(0.2);
    private static final Voltage kDriveFrictionVoltage = Volts.of(0.2);


    public static final SwerveDrivetrainConstants constants
     = new SwerveDrivetrainConstants()
        .withCANBusName(bus.getName())
        .withPigeon2Configs(GyroConfig)
        .withPigeon2Id(0);
    
    public static final SwerveModuleConstantsFactory<TalonFXConfiguration, TalonFXConfiguration, CANcoderConfiguration> ModuleCreator = new SwerveModuleConstantsFactory<TalonFXConfiguration, TalonFXConfiguration, CANcoderConfiguration>()
    .withDriveMotorGearRatio(DriveGearRatio)
    .withSteerMotorGearRatio(SteerGearRatio)
    .withCouplingGearRatio(CoupleGearRatio)
    .withWheelRadius(WheelRadius)
    .withSteerMotorGains(SteerPID)
    .withDriveMotorGains(DrivePID)
    .withSteerMotorClosedLoopOutput(SteerOutput)
    .withDriveMotorClosedLoopOutput(DriveOutput)
    .withSlipCurrent(SlipCurrent)
    .withSpeedAt12Volts(MaxVelocity)
    .withDriveMotorType(DriveMotor)
    .withSteerMotorType(SteerMotor) 
    .withFeedbackSource(SteerEncoder)
    .withDriveMotorInitialConfigs(DriveConfig)
    .withSteerMotorInitialConfigs(SteerConfig)
    .withEncoderInitialConfigs(EncoderConfig)
    .withSteerInertia(kSteerInertia)
    .withDriveInertia(kDriveInertia)
    .withSteerFrictionVoltage(kSteerFrictionVoltage)
    .withDriveFrictionVoltage(kDriveFrictionVoltage);

    public class Modules {
        public static final boolean LeftSideInverted = false;
        public static final boolean RightSideInverted = true;
        public static final double OffsetValue = Centimeters.of(65).div(2).in(Meters);
        public static final double TuningDelta = -0.25;
        public static final boolean SteerMotorInverted = true;
        public static final boolean SteerEncoderInverted = false;

        public class FrontLeft {
            public static final int DriveID = 11;
            public static final int SteerID = 12;
            public static final int EncoderID = 10;
            public static final Angle Offset = Rotations.of(-0.130615234375);
            public static final Translation2d place = new Translation2d(-OffsetValue, OffsetValue);
            public static final SwerveModuleConstants<TalonFXConfiguration, TalonFXConfiguration, CANcoderConfiguration> constant = ModuleCreator.createModuleConstants(
                SteerID, DriveID, EncoderID, Offset,
            place.getMeasureX(), place.getMeasureY(), LeftSideInverted, SteerMotorInverted, SteerEncoderInverted
            );
        }
        
        public class FrontRight {
            public static final int DriveID = 21;
            public static final int SteerID = 22;
            public static final int EncoderID = 2;
            public static final Angle Offset = Rotations.of(-0.098388671875);
            public static final Translation2d place = new Translation2d(OffsetValue, OffsetValue);
            public static final SwerveModuleConstants<TalonFXConfiguration, TalonFXConfiguration, CANcoderConfiguration> constant = ModuleCreator.createModuleConstants(
                SteerID, DriveID, EncoderID, Offset,
            place.getMeasureX(), place.getMeasureY(), false, SteerMotorInverted, SteerEncoderInverted
            );
        }
        

        public class BackLeft {
            public static final int DriveID = 31;
            public static final int SteerID = 32;
            public static final int EncoderID = 3;
            public static final Angle Offset = Rotations.of(0.08984375);
            public static final Translation2d place = new Translation2d(-OffsetValue, -OffsetValue);
            public static final SwerveModuleConstants<TalonFXConfiguration, TalonFXConfiguration, CANcoderConfiguration> constant = ModuleCreator.createModuleConstants(
                SteerID, DriveID, EncoderID, Offset,
            place.getMeasureX(), place.getMeasureY(), true, SteerMotorInverted, SteerEncoderInverted);
        }
        

        public class BackRight {
            public static final int DriveID = 41;
            public static final int SteerID = 42;
            public static final int EncoderID = 4;
            public static final Angle Offset = Rotations.of(0.292236328125);
            public static final Translation2d place = new Translation2d(OffsetValue, -OffsetValue);
            public static final SwerveModuleConstants<TalonFXConfiguration, TalonFXConfiguration, CANcoderConfiguration> constant = ModuleCreator.createModuleConstants(
                SteerID, DriveID, EncoderID, Offset,
            place.getMeasureX(), place.getMeasureY(), RightSideInverted, SteerMotorInverted, SteerEncoderInverted
            );
        }
        
    }
}
