package frc.robot.Shooter;

import com.ctre.phoenix6.configs.CANcoderConfiguration;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.MotionMagicVelocityVoltage;
import com.ctre.phoenix6.controls.MotionMagicVoltage;
import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.hardware.TalonFX;

import edu.wpi.first.wpilibj.Counter;
import edu.wpi.first.wpilibj.DigitalInput;
import frc.robot.Shooter.Constants.Pitch;
import frc.robot.Shooter.Constants.Shoot;

public class Shooter {
    public TalonFX PitchingMotor, ShootingMotor;
    public CANcoder PitchingEncoder;

    public MotionMagicVoltage PitchingPID;
    public MotionMagicVelocityVoltage ShootPID;
    public DigitalInput hasBall, ballCounterInput;
    public Counter ballCounter;

    private TalonFXConfiguration PitchConfig, ShootConfig;
    private CANcoderConfiguration PitchEncoderConfig;

    private static Shooter inst;

    private Shooter(){
        PitchingMotor = new TalonFX(Pitch.MotorID, Constants.bus);
        ShootingMotor = new TalonFX(Shoot.MotorID, Constants.bus);
        PitchingEncoder = new CANcoder(Pitch.MotorEncoderID, Constants.bus);

        PitchingPID = new MotionMagicVoltage(0);
        ShootPID = new MotionMagicVelocityVoltage(0);
        hasBall = new DigitalInput(0);
        ballCounterInput = new DigitalInput(0);

        PitchConfig = new TalonFXConfiguration();
        ShootConfig = new TalonFXConfiguration();
        PitchEncoderConfig = new CANcoderConfiguration();

        
    }

    public static Shooter getInstance(){
        inst = inst == null ? new Shooter() : inst;
        return inst;
    }
}
