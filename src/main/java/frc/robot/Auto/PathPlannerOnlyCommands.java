package frc.robot.Auto;

import static edu.wpi.first.units.Units.Seconds;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.units.measure.Time;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Drivetrain.Drivetrain;
import frc.robot.Shooter.Shooter;

public class PathPlannerOnlyCommands {
    public static Shooter shooter = Shooter.getInstance();

    public static Command shootShortWhile(){
        Command shootingCommand = shoot(new SwerveModuleState(17, Rotation2d.fromDegrees(65)), Seconds.of(3));
        shootingCommand.addRequirements(Drivetrain.getInstance());
        return shootingCommand;
    }

    public static Command shootLongWhile(){
        Command shootingCommand = shoot(new SwerveModuleState(17, Rotation2d.fromDegrees(65)), Seconds.of(8));
        shootingCommand.addRequirements(Drivetrain.getInstance());
        return shootingCommand;
    }

    private static Command shoot(SwerveModuleState state, Time duration){
        return shooter.setState(state).withTimeout(duration);
    }
}
