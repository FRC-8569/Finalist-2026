package frc.robot.Shooter;

import static edu.wpi.first.units.Units.Celsius;
import static edu.wpi.first.units.Units.Meters;
import static edu.wpi.first.units.Units.MetersPerSecond;
import static edu.wpi.first.units.Units.RadiansPerSecond;

import java.util.function.Supplier;

import com.ctre.phoenix6.configs.CANcoderConfiguration;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.DutyCycleOut;
import com.ctre.phoenix6.controls.MotionMagicVelocityTorqueCurrentFOC;
import com.ctre.phoenix6.controls.MotionMagicVoltage;
import com.ctre.phoenix6.controls.NeutralOut;
import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import com.ctre.phoenix6.signals.SensorDirectionValue;

import dev.doglog.DogLog;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.LinearVelocity;
import edu.wpi.first.wpilibj.Alert;
import edu.wpi.first.wpilibj.Alert.AlertType;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.Subsystem;
import frc.robot.Shooter.Constants.Pitch;
import frc.robot.Shooter.Constants.Shoot;

public class Shooter implements Subsystem {
    public TalonFX ShootingMotor, PitchingMotor;
    public CANcoder PitchingEncoder;

    public MotionMagicVoltage PitchingPID;
    public MotionMagicVelocityTorqueCurrentFOC ShootPID;
    public DutyCycleOut IndexPID, SpindexPID;

    private TalonFXConfiguration PitchConfig, ShootConfig, IndexConfig, SpindexConfig;
    private CANcoderConfiguration PitchEncoderConfig;
    public SwerveModuleState targetState = new SwerveModuleState(0, Rotation2d.kZero);
    
    public Alert ShooterAlert;

    private static Shooter inst;
    private Shooter(){
        PitchingMotor = new TalonFX(Pitch.MotorID, Constants.bus);
        ShootingMotor = new TalonFX(Shoot.MotorID, Constants.bus);
        PitchingEncoder = new CANcoder(Pitch.EncoderID, Constants.bus);
    
        PitchingPID = new MotionMagicVoltage(0);
        ShootPID = new MotionMagicVelocityTorqueCurrentFOC(0);
        IndexPID = new DutyCycleOut(0);
        SpindexPID = new DutyCycleOut(0);

        ShooterAlert = new Alert("Robot", "Shooter Overheated", AlertType.kWarning);

        ShootConfig = new TalonFXConfiguration();
        PitchConfig = new TalonFXConfiguration();
        IndexConfig = new TalonFXConfiguration();
        SpindexConfig = new TalonFXConfiguration();
        PitchEncoderConfig = new CANcoderConfiguration();

        PitchConfig.MotorOutput
            .withNeutralMode(NeutralModeValue.Brake)
            .withInverted(InvertedValue.Clockwise_Positive);
        PitchConfig.Feedback
            .withSensorToMechanismRatio(10);
        PitchConfig.withSlot0(Pitch.PitchPID);
        PitchConfig.withMotionMagic(Pitch.PitchMagic);
        PitchConfig.withSoftwareLimitSwitch(Pitch.PitchLimit);

        ShootConfig.MotorOutput
            .withNeutralMode(NeutralModeValue.Coast)
            .withInverted(InvertedValue.Clockwise_Positive);
        ShootConfig.Feedback
            .withSensorToMechanismRatio(Shoot.GearRatio);
        ShootConfig.MotionMagic
            .withMotionMagicAcceleration(25);

        IndexConfig.MotorOutput
            .withInverted(InvertedValue.Clockwise_Positive)
            .withNeutralMode(NeutralModeValue.Coast);

        SpindexConfig.MotorOutput
            .withInverted(InvertedValue.Clockwise_Positive)
            .withNeutralMode(NeutralModeValue.Coast);

        PitchEncoderConfig.MagnetSensor
            .withSensorDirection(SensorDirectionValue.Clockwise_Positive)
            .withMagnetOffset(Pitch.PitchOffset);

        ShootConfig.withSlot0(Shoot.ShootPID);
        ShootConfig.withMotionMagic(Shoot.ShootMagic);
        PitchingEncoder.getConfigurator().apply(PitchEncoderConfig);

        PitchingMotor.getConfigurator().apply(PitchConfig);
        ShootingMotor.getConfigurator().apply(ShootConfig);

        PitchingMotor.setPosition(0);

        register();

        setDefaultCommand(setState(() -> new SwerveModuleState(MetersPerSecond.of(1), getState().angle)));
    }

    public SwerveModuleState getState(){
        return new SwerveModuleState(
            MetersPerSecond.of(ShootingMotor.getVelocity().getValue().in(RadiansPerSecond)*Shoot.WheelRadius.in(Meters)),
            new Rotation2d(PitchingEncoder.getPosition().getValue())
        );
    }

    public Command setState(SwerveModuleState state){
        if(state.angle.getMeasure().lt(Pitch.PitchingAngle.getFirst()) || state.angle.getMeasure().gt(Pitch.PitchingAngle.getSecond()) || state.speedMetersPerSecond > Shoot.MaxVelocity.in(MetersPerSecond)) return idle();
        else return run(() -> {
            PitchingMotor.setControl(PitchingPID.withPosition(state.angle.getMeasure()));
            ShootingMotor.setControl(ShootPID.withVelocity(RadiansPerSecond.of(state.speedMetersPerSecond/Shoot.WheelRadius.in(Meters))));
            targetState = state;
        }).until(() -> PitchingMotor.getPosition().getValue().isNear(PitchingPID.getPositionMeasure(), 0.05) && ShootingMotor.getPosition().getValue().isNear(ShootPID.getVelocityMeasure(), 0.05));
    }

    private Command setState(Supplier<SwerveModuleState> state){
        return setState(state.get());
    }

    public Command setShooterState(LinearVelocity vel){
        return run(() -> ShootingMotor.setControl(ShootPID.withVelocity(RadiansPerSecond.of(vel.in(MetersPerSecond)/Shoot.WheelRadius.in(Meters)))))
            .until(() -> ShootingMotor.getVelocity().getValue().isNear(ShootPID.getVelocityMeasure(), 0.05));
            
    }

    public Command pitchShooter(Angle angle){
        if(angle.lt(Pitch.PitchingAngle.getFirst()) || angle.gt(Pitch.PitchingAngle.getSecond())) return idle();
        return run(() -> {
            PitchingMotor.setControl(PitchingPID.withPosition(angle));
        })
        .handleInterrupt(() -> PitchingMotor.setControl(new NeutralOut()));
    }

    public Command stopShooter(){
        return runOnce(() -> ShootingMotor.stopMotor());
    }
    
    public Command resetPitch(){
        return Commands.runOnce(() -> PitchingMotor.setPosition(0));
    }

    @Override
    public void periodic(){
        DogLog.log("Debug/Shooter/PitchingMotorState", PitchingMotor.getControlMode().toString());
        DogLog.log("Debug/Shooter/PitchTarget", PitchingPID.getPositionMeasure());
        DogLog.log("Debug/Shooter/PitchingAngle", PitchingMotor.getPosition().getValue());
        DogLog.log("Driver/Shooter/TargetState", targetState);
        DogLog.log("Driver/Shooter/CurrentState", getState());

        ShooterAlert.set(ShootingMotor.getDeviceTemp().getValue().gt(Celsius.of(60)));
    }

    public static Shooter getInstance(){
        inst = inst == null ? new Shooter() : inst;
        return inst;
    }
}
