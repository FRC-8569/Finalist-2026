package frc.robot.Shooter;

import static edu.wpi.first.units.Units.Celsius;
import static edu.wpi.first.units.Units.Degrees;
import static edu.wpi.first.units.Units.Meters;
import static edu.wpi.first.units.Units.MetersPerSecond;
import static edu.wpi.first.units.Units.Newton;
import static edu.wpi.first.units.Units.RadiansPerSecond;
import java.util.List;
import java.util.function.Supplier;

import com.ctre.phoenix6.configs.CANcoderConfiguration;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.DutyCycleOut;
import com.ctre.phoenix6.controls.MotionMagicVelocityTorqueCurrentFOC;
import com.ctre.phoenix6.controls.PositionVoltage;
import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import com.ctre.phoenix6.signals.SensorDirectionValue;

import dev.doglog.DogLog;
import edu.wpi.first.math.Pair;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.units.measure.LinearVelocity;
import edu.wpi.first.wpilibj.Alert;
import edu.wpi.first.wpilibj.Alert.AlertType;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.Subsystem;
import frc.robot.Shooter.Constants.Pitch;
import frc.robot.Shooter.Constants.Shoot;
import frc.utils.Tools.RobotState;

public class Shooter implements Subsystem {
    public TalonFX ShootingMotor, PitchingMotor;

    public PositionVoltage PitchingPID;
    public MotionMagicVelocityTorqueCurrentFOC ShootPID;
    public DutyCycleOut IndexPID, SpindexPID;

    private TalonFXConfiguration PitchConfig, ShootConfig;
    private CANcoderConfiguration PitchEncoderConfig;
    public SwerveModuleState targetState = new SwerveModuleState(0, Rotation2d.kZero);
    
    public Alert ShootTempAlert, ShootStallingError;
    public boolean isStalled = false;

    private static Shooter inst;
    private Shooter(){
        PitchingMotor = new TalonFX(Pitch.MotorID, Constants.bus);
        ShootingMotor = new TalonFX(Shoot.MotorID, Constants.bus);
    
        PitchingPID = new PositionVoltage(0);
        ShootPID = new MotionMagicVelocityTorqueCurrentFOC(0);
        IndexPID = new DutyCycleOut(0);
        SpindexPID = new DutyCycleOut(0);

        ShootTempAlert = new Alert("Robot", "Shooter Overheated", AlertType.kWarning);
        ShootStallingError = new Alert("Robot", "ShootStallingError", AlertType.kWarning);

        ShootConfig = new TalonFXConfiguration();
        PitchConfig = new TalonFXConfiguration();
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
            .withInverted(InvertedValue.CounterClockwise_Positive);
        ShootConfig.Feedback
            .withSensorToMechanismRatio(Shoot.GearRatio);

        PitchEncoderConfig.MagnetSensor
            .withSensorDirection(SensorDirectionValue.Clockwise_Positive)
            .withMagnetOffset(Pitch.PitchOffset);

        ShootConfig.withSlot0(Shoot.ShootPID);
        ShootConfig.withMotionMagic(Shoot.ShootMagic);
        
        PitchingMotor.getConfigurator().apply(PitchConfig);
        ShootingMotor.getConfigurator().apply(ShootConfig);

        register();
    }

    public SwerveModuleState getState(){
        return new SwerveModuleState(
            MetersPerSecond.of(ShootingMotor.getVelocity().getValue().in(RadiansPerSecond)*Shoot.WheelRadius.in(Meters)),
            new Rotation2d(PitchingMotor.getPosition().getValue())
        );
    }

    public Command setState(SwerveModuleState state){
        return run(() -> {
            PitchingMotor.setControl(PitchingPID.withPosition(Degrees.of(90).minus(state.angle.getMeasure())));
            ShootingMotor.setControl(ShootPID.withVelocity(RadiansPerSecond.of(state.speedMetersPerSecond/Shoot.WheelRadius.in(Meters))));
            targetState = state;
        }).withName("Shooting in %.2f m/s %.2f degree".formatted(state.speedMetersPerSecond, state.angle.getDegrees()));
    }

    public Command setState(RobotState state){
        return setState(state.getShootingState());
    }

    public Command setState(Supplier<SwerveModuleState> state){
        return setState(state.get());
    }

    public Command setShooterState(LinearVelocity vel){
        return run(() -> ShootingMotor.setControl(ShootPID.withVelocity(RadiansPerSecond.of(vel.in(MetersPerSecond)/Shoot.WheelRadius.in(Meters)))))
            .until(() -> ShootingMotor.getVelocity().getValue().isNear(ShootPID.getVelocityMeasure(), 0.05))
            .handleInterrupt(() -> ShootingMotor.stopMotor());
    }

    public Command calibrateShooter(){
        return runOnce(() -> PitchingMotor.setPosition(Degrees.of(25.6))).ignoringDisable(true);
        // return runEnd(() -> {
        //     PitchingMotor.getConfigurator().apply(Pitch.PitchLimit.withReverseSoftLimitEnable(false));
        //     PitchingMotor.set(-0.1);
        //     }, () -> {
        //         PitchingMotor.stopMotor();
        //         PitchEncoder.setPosition(Degrees.of(256));
        //         PitchingMotor.getConfigurator().apply(Pitch.PitchLimit.withReverseSoftLimitEnable(true));
        //     }).until(() -> PitchingMotor.getStatorCurrent().getValue().gt(Amps.of(30)));
    }

    public Command stopShooter(){
        return runOnce(() -> ShootingMotor.stopMotor());
    }
    
    public Command resetPitch(){
        return Commands.runOnce(() -> PitchingMotor.setPosition(0)).ignoringDisable(true);
    }

    @Override
    public void periodic(){
        DogLog.log("Debug/Shooter/PitchingMotorState", PitchingMotor.getControlMode().toString());
        DogLog.log("Debug/Shooter/PitchTarget", PitchingPID.getPositionMeasure());
        DogLog.log("Debug/Shooter/PitchingAngle", PitchingMotor.getPosition().getValue());
        DogLog.log("Debug/Shooter/ShooterForce", (ShootingMotor.getStatorCurrent().getValueAsDouble()*ShootingMotor.getMotorKT().getValueAsDouble())/Shoot.WheelRadius.in(Meters), Newton);
        DogLog.log("Debug/Shooter/ShootEnergyCost", ShootingMotor.getSupplyCurrent().getValue().times(ShootingMotor.getSupplyVoltage().getValue()));
        DogLog.log("Driver/Shooter/TargetState", targetState);
        DogLog.log("Driver/Shooter/CurrentState", getState());
        if(getCurrentCommand() != null) DogLog.log("Debug/Shooter/CurrentCommand", getCurrentCommand().getName());
      

        ShootTempAlert.set(ShootingMotor.getDeviceTemp().getValue().gt(Celsius.of(60)));
        ShootStallingError.set((ShootingMotor.getStatorCurrent().getValueAsDouble()*ShootingMotor.getMotorKT().getValueAsDouble())/Shoot.WheelRadius.in(Meters) > 20);
    }

    public List<Pair<Integer, Boolean>> isConnected(){
        return List.of(
            Pair.of(PitchingMotor.getDeviceID(), PitchingMotor.isConnected()),
            Pair.of(ShootingMotor.getDeviceID(), PitchingMotor.isConnected())
        );
    }

    public static Shooter getInstance(){
        inst = inst == null ? new Shooter() : inst;
        return inst;
    }
}
