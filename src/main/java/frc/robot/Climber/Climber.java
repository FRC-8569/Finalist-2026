package frc.robot.Climber;

import static edu.wpi.first.units.Units.NewtonMeter;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.DutyCycleOut;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;

import dev.doglog.DogLog;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Subsystem;

public class Climber implements Subsystem{
    public TalonFX ClimbMotor;

    public DutyCycleOut ClimbPID;

    private TalonFXConfiguration ClimbConfig;
    private static Climber inst;

    private Climber(){
        ClimbMotor = new TalonFX(Constants.MotorID, Constants.bus);

        ClimbPID = new DutyCycleOut(0);

        ClimbConfig = new TalonFXConfiguration();

        ClimbConfig.MotorOutput
            .withNeutralMode(NeutralModeValue.Brake)
            .withInverted(InvertedValue.CounterClockwise_Positive);

        ClimbMotor.getConfigurator().apply(ClimbConfig);
        register();
    }

    public Command climb(double percent){
        return runEnd(
            () -> ClimbMotor.set(percent), 
            () -> ClimbMotor.stopMotor());
    }

    @Override
    public void periodic(){
        DogLog.log("Driver/Climber/ClimbPosition", ClimbMotor.getPosition().getValue());
        DogLog.log("Driver/Climber/ClimbVelocity", ClimbMotor.getVelocity().getValue());
        DogLog.log("Debug/Climber/ClimbForce", ClimbMotor.getStatorCurrent().getValueAsDouble()*ClimbMotor.getMotorKT().getValueAsDouble(), NewtonMeter);
    }

    public static Climber getInstance(){
        inst = inst == null ? new Climber() : inst;
        return inst;
    }
}
