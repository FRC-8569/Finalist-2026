package frc.robot.Intake;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.MotionMagicExpoTorqueCurrentFOC;
import com.ctre.phoenix6.controls.MotionMagicTorqueCurrentFOC;
import com.ctre.phoenix6.controls.MotionMagicVelocityTorqueCurrentFOC;
import com.ctre.phoenix6.hardware.TalonFX;

import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Intake.Constants.Roll;
import frc.robot.Intake.Constants.Tongue;

public class Intake {
    public TalonFX TongueMotor, RollMotor;

    public MotionMagicExpoTorqueCurrentFOC TonguePID;
    public MotionMagicVelocityTorqueCurrentFOC RollPID;
    private boolean hasStalled = false;

    private TalonFXConfiguration TongueConfig, RollConfig;
    private static Intake inst;
    
    private Intake(){
        TongueMotor = new TalonFX(Tongue.MotorID, Constants.bus);
        RollMotor = new TalonFX(Roll.MotorID, Constants.bus);

        TonguePID = new MotionMagicExpoTorqueCurrentFOC(0);
        RollPID = new MotionMagicVelocityTorqueCurrentFOC(0);

        TongueConfig = new TalonFXConfiguration();
        RollConfig = new TalonFXConfiguration();
    }

    public Angle getIntakePosition(){
        return TongueMotor.getPosition().getValue();
    }

    public static Intake getInstance(){
        inst = inst == null ? new Intake() : inst;
        return inst;
    }
}
