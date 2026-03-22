package frc.robot.Auto;

import static edu.wpi.first.units.Units.Seconds;

import java.util.Optional;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.units.measure.Time;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.Drivetrain.Drivetrain;
import frc.robot.Shooter.Shooter;
import frc.utils.Tools.FieldSide;

public class Auto {
    private static Drivetrain drivetrain = Drivetrain.getInstance();
    private static Shooter shooter = Shooter.getInstance();
    public static Optional<Command> getAutoCommand(FieldSide side){
        if(side != null){
            return Optional.of(Commands.none());
        }else{
            if(side == FieldSide.RIGHT){
                return Optional.of(new SequentialCommandGroup(
                    drivetrain.drive("RightBumpToShootPose"),
                    shooter.setState(new SwerveModuleState(19, Rotation2d.fromDegrees(63))).withTimeout(Seconds.of(3)),
                    drivetrain.drive("RightBumpToOutpost"),
                    new WaitCommand(Seconds.of(5)),
                    drivetrain.drive("OutpostToRightBump"),
                    shooter.setState(new SwerveModuleState(19, Rotation2d.fromDegrees(63)))
                ));
            }else{
                return Optional.of(new SequentialCommandGroup(
                    drivetrain.drive("LeftBumpToShootPose"),
                    shooter.setState(new SwerveModuleState(19, Rotation2d.fromDegrees(63))).withTimeout(Seconds.of(5))
                )
                );
            }
        }
    }
}
