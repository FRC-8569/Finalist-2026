package frc.robot.Auto;

import static edu.wpi.first.units.Units.MetersPerSecond;
import static edu.wpi.first.units.Units.Seconds;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.GlobalConstant;
import frc.robot.Drivetrain.Drivetrain;

public class Auto {
    public static Drivetrain drivetrain = Drivetrain.getInstance();
    public static Timer timer = new Timer();
    public static Command getAutoCommand(){
        return new SequentialCommandGroup(
            Commands.runOnce(() -> {timer.reset();timer.start();}),
            drivetrain.drive(new Pose2d(14,6, Rotation2d.fromDegrees(-135)),MetersPerSecond.of(0)),
            new WaitCommand(Seconds.of(3)),
            drivetrain.driveOverTrench(),
            drivetrain.drive(new Pose2d(10,7, Rotation2d.kCW_90deg), MetersPerSecond.of(0)),
            drivetrain.drive(new Pose2d(10, 1, Rotation2d.kCW_90deg), MetersPerSecond.of(1)),
            drivetrain.driveOverTrench(),
            drivetrain.drive(new Pose2d(13.5,1, Rotation2d.fromDegrees(120)), MetersPerSecond.of(0)),
            new WaitCommand(3),
            drivetrain.driveToTower()
            // drivetrain.drive(new Pose2d(14,6, Rotation2d.fromDegrees(-135))),
            // new WaitCommand(Seconds.of(3)),
            // drivetrain.drive(MetersPerSecond.of(2), GlobalConstant.RightTrench,new Pose2d(9,7, Rotation2d.kCW_90deg),new Pose2d(9, 1, Rotation2d.kCW_90deg)),
            // drivetrain.drive(GlobalConstant.LeftTrench,MetersPerSecond.of(2)),
            // drivetrain.drive(new Pose2d(13.5,1, Rotation2d.fromDegrees(120))),
            // new WaitCommand(Seconds.of(3)),
            // drivetrain.drive(GlobalConstant.LeftTrench,MetersPerSecond.of(2)),
            // drivetrain.drive(new Pose2d(9, 1, Rotation2d.kCW_90deg),MetersPerSecond.of(2)),
            // drivetrain.drive(new Pose2d(9,7, Rotation2d.kCW_90deg),MetersPerSecond.of(2)),
            // drivetrain.drive(GlobalConstant.RightTrench,MetersPerSecond.of(2)),
            // drivetrain.drive(new Pose2d(15,4,Rotation2d.kZero))
        ).beforeStarting(drivetrain.drive(new Pose2d(13,7.3, Rotation2d.k180deg),MetersPerSecond.of(0)).andThen(new WaitCommand(Seconds.of(3))).beforeStarting(drivetrain.fieldReset()));
    }
}
