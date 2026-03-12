package frc.robot.Auto;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.Drivetrain.Drivetrain;
import frc.robot.Intake.Intake;
import frc.robot.Shooter.Shooter;
import frc.utils.Tools.FieldSide;

public class Auto {
    public static Drivetrain drivetrain = Drivetrain.getInstance();
    public static Intake intake = Intake.getInstance();
    public static Shooter shooter = Shooter.getInstance();
    public static Timer timer = new Timer();

    public static Command getAutoCommand(){
        FieldSide side = drivetrain.getSide();
        return new SequentialCommandGroup(
            drivetrain.drive(new Pose2d(13.1,4,Rotation2d.kZero), side),
            new WaitCommand(3),
            drivetrain.drive(new Pose2d(16,7.4,Rotation2d.kCCW_90deg), FieldSide.LEFT)
            // drivetrain.drive(new Pose2d(9,1.2,Rotation2d.kCCW_90deg), side),
            // drivetrain.drive(new Pose2d(9,3.6, Rotation2d.kCCW_90deg), side),
            // drivetrain.driveToShootPose(),
            // drivetrain.drive(new Pose2d(16.1,7.5,Rotation2d.kCCW_90deg), Side.LEFT).onlyIf(() -> side == Side.LEFT)
        );
    }
}
