package frc.robot.Intake;

import static edu.wpi.first.units.Units.Amps;
import static edu.wpi.first.units.Units.Meters;
import static edu.wpi.first.units.Units.MetersPerSecond;
import static edu.wpi.first.units.Units.RadiansPerSecond;
import static edu.wpi.first.units.Units.RotationsPerSecond;

import java.util.List;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.CoastOut;
import com.ctre.phoenix6.controls.DutyCycleOut;
import com.ctre.phoenix6.controls.MotionMagicExpoTorqueCurrentFOC;
import com.ctre.phoenix6.controls.MotionMagicVelocityVoltage;
import com.ctre.phoenix6.controls.StaticBrake;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.ControlModeValue;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;

import dev.doglog.DogLog;
import edu.wpi.first.math.Pair;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.LinearVelocity;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.simulation.FlywheelSim;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.Subsystem;
import frc.robot.Drivetrain.Drivetrain;
import frc.robot.Intake.Constants.Roller;
import frc.robot.Intake.Constants.Tongue;

public class Intake implements Subsystem{
    public TalonFX TongueMotor, RollMotor;
    public MotionMagicExpoTorqueCurrentFOC TonguePID;
    public MotionMagicVelocityVoltage RollingPID;
    public DutyCycleOut SpindexPID;
    public TalonFXConfiguration TongueConfig, RollConfig, SpindexConfig;
    public Timer DebounceTimer;

    public FlywheelSim TongueSim, RollSim;

    private boolean hasStalled = false;
    private static Intake inst;

    private Intake(){
        TongueMotor = new TalonFX(Tongue.MotorID, Constants.bus);
        RollMotor = new TalonFX(Roller.MotorID, Constants.bus);

        TonguePID = new MotionMagicExpoTorqueCurrentFOC(0).withFeedForward(0.15);
        RollingPID = new MotionMagicVelocityVoltage(0);
        SpindexPID = new DutyCycleOut(0);

        DebounceTimer = new Timer();

        TongueConfig = new TalonFXConfiguration();
        RollConfig = new TalonFXConfiguration();
        SpindexConfig = new TalonFXConfiguration();

        TongueConfig.MotorOutput
            .withInverted(InvertedValue.Clockwise_Positive)
            .withNeutralMode(NeutralModeValue.Brake);
        TongueConfig.Feedback
            .withSensorToMechanismRatio(Tongue.GearRatio);
        TongueConfig.withSlot0(Tongue.TonguePID);
        TongueConfig.withMotionMagic(Tongue.TongueMagic);
        TongueConfig.withSoftwareLimitSwitch(Tongue.TongueLimit);
        // TongueConfig.withCurrentLimits(Tongue.TongueCurrentLimit);

        RollConfig.MotorOutput
            .withInverted(InvertedValue.CounterClockwise_Positive)
            .withNeutralMode(NeutralModeValue.Coast);
        RollConfig.Feedback
            .withSensorToMechanismRatio(Roller.GearRatio);
        RollConfig.withSlot0(Roller.RollerPID);
        RollConfig.withMotionMagic(Roller.RollerMagic);

        SpindexConfig.MotorOutput
            .withInverted(InvertedValue.Clockwise_Positive)
            .withNeutralMode(NeutralModeValue.Coast);

        TongueMotor.getConfigurator().apply(TongueConfig);
        RollMotor.getConfigurator().apply(RollConfig);
        register();
    }

    public Angle getIntakePosition(){
        return TongueMotor.getPosition().getValue();
    }

    public LinearVelocity getRollerVelocity(){
        return MetersPerSecond.of(RollMotor.getVelocity().getValueAsDouble()*Roller.WheelRadius.in(Meters));
    }

     public Command moveIntake(boolean isOut){
        return run(() -> {
            TongueMotor.setControl(TonguePID.withPosition(isOut ? 1.4 : 0));
        }).until(() -> TongueMotor.getVelocity().getValue().abs(RotationsPerSecond) < 0.01 && getIntakePosition().isNear(TonguePID.getPositionMeasure(), 0.1))
            .finallyDo(() -> {
                TongueMotor.setControl(isOut ? new CoastOut(): new StaticBrake());
            }).withName("MovingIntake").beforeStarting(CalibrateIntake().onlyIf(() -> hasStalled));
    }

    public Command intake(boolean isIntake){
        double v = Math.hypot(Drivetrain.getInstance().getState().Speeds.vxMetersPerSecond, Drivetrain.getInstance().getState().Speeds.vyMetersPerSecond);
        return setRollVelocity(MetersPerSecond.of(v).times(2));
    }

    public Command setRollVelocity(LinearVelocity velocity){
        double v = Math.max(Roller.SpeedRange.getFirst().in(MetersPerSecond), Math.min(Roller.SpeedRange.getSecond().in(MetersPerSecond), velocity.in(MetersPerSecond)));
        return run(() -> {
            RollMotor.setControl(RollingPID.withVelocity(RadiansPerSecond.of(v/Roller.WheelRadius.in(Meters))));
            if(RollMotor.getStatorCurrent().getValue().gt(Amps.of(80))){
                RollMotor.set(-0.1);
                try{
                    Thread.sleep(100);
                }catch(Exception e){}
                RollMotor.setControl(RollingPID.withVelocity(RadiansPerSecond.of(v/Roller.WheelRadius.in(Meters))));
            }
        })
                .handleInterrupt(() -> RollMotor.stopMotor());
    }

    public Command CalibrateIntake(){
        return runEnd(() -> {
            TongueMotor.getConfigurator().apply(Tongue.TongueLimit.withReverseSoftLimitEnable(false));
            TongueMotor.set(-0.2);
            }, () -> {
            TongueMotor.stopMotor();
            DebounceTimer.stop();
            TongueMotor.setPosition(0);
            TongueMotor.getConfigurator().apply(Tongue.TongueLimit.withReverseSoftLimitEnable(true));
            hasStalled = false;

        }).until(() -> TongueMotor.getStatorCurrent().getValue().gt(Amps.of(11.5)) && DebounceTimer.hasElapsed(0.2)).beforeStarting(Commands.runOnce(() -> {
            DebounceTimer.reset();
            DebounceTimer.start();
        })).withName("Calibrating");
    }

    @Override
    public void periodic(){
        if(DriverStation.isDisabled() && TongueMotor.getControlMode().getValue() != ControlModeValue.CoastOut) TongueMotor.setControl(new CoastOut());
        // 
        
        log();

    }


    public void log(){
        DogLog.log("Driver/Intake/IntakePosition", getIntakePosition());
        DogLog.log("Driver/Intake/TongueHasStalled", hasStalled);
        DogLog.log("Debug/Intake/TongueAmps", TongueMotor.getStatorCurrent().getValue());
        DogLog.log("Debug/Intake/TongueControlMode", TongueMotor.getControlMode().getValue().toString());
        DogLog.log("Debug/Intake/RollerCurrent", RollMotor.getStatorCurrent().getValue());
        DogLog.log("Debug/Intake/DebounceTiemr", DebounceTimer.get());
    }

    public List<Pair<Integer, Boolean>> isConnected(){
        return List.of(
            Pair.of(TongueMotor.getDeviceID(), TongueMotor.isConnected()),
            Pair.of(RollMotor.getDeviceID(), RollMotor.isConnected())
        );
    }

    public static Intake getInstance(){
        inst = inst == null ? new Intake() : inst;
        return inst;
    }
}
