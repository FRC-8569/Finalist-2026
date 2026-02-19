package frc.robot.Auto;

import static edu.wpi.first.units.Units.MetersPerSecond;
import static edu.wpi.first.units.Units.Seconds;

import com.ctre.phoenix6.Utils;

import dev.doglog.DogLog;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.Drivetrain.Drivetrain;
import frc.robot.Intake.Intake;
import frc.robot.Shooter.Shooter;

public class Auto {
    public static Drivetrain drivetrain = Drivetrain.getInstance();
    public static Intake intake = Intake.getInstance();
    public static Shooter shooter = Shooter.getInstance();
    public static Timer timer = new Timer();
    public static Command getAutoCommand(){
        return new SequentialCommandGroup(
            Commands.runOnce(() -> {timer.reset();timer.start();}),
            // drivetrain.driveOverTrench().alongWith(intake.moveIntake(IntakePosition.Out)),
            drivetrain.drive(new Pose2d(10,7, Rotation2d.kCCW_90deg), MetersPerSecond.of(0)),
            drivetrain.drive(new Pose2d(10, 5, Rotation2d.kCCW_90deg), MetersPerSecond.of(0.5)).raceWith(intake.intake(true)),
            drivetrain.driveOverTrench(),
            drivetrain.drive(new Pose2d(13,7.5, Rotation2d.fromDegrees(60)), MetersPerSecond.of(0)),
            drivetrain.driveToTower()
            // 
            // drivetrain.drive(new Pose2d(14,6, Rotation2d.fromDegrees(-135)),MetersPerSecond.of(0)),
            // new WaitCommand(Seconds.of(3)),
            // drivetrain.driveOverTrench().alongWith(intake.moveIntake(IntakePosition.Out)),
            // drivetrain.drive(new Pose2d(10,7, Rotation2d.kCCW_90deg), MetersPerSecond.of(0)),
            // drivetrain.drive(new Pose2d(10, 1, Rotation2d.kCCW_90deg), MetersPerSecond.of(0.5)).raceWith(intake.intake(true)),
            // drivetrain.driveOverTrench().alongWith(intake.moveIntake(IntakePosition.Idle)),
            // drivetrain.drive(new Pose2d(13.5,1, Rotation2d.fromDegrees(120)), MetersPerSecond.of(0)),
            // new WaitCommand(3),
            // drivetrain.driveToTower()
        ).alongWith(Commands.run(() -> DogLog.log("Utils/AutoTime", 20-timer.get())).onlyIf(() -> Utils.isSimulation()))
        .beforeStarting(drivetrain.drive(new Pose2d(13,7.3, Rotation2d.k180deg),MetersPerSecond.of(0)).andThen(new WaitCommand(Seconds.of(3))).beforeStarting(drivetrain.fieldReset().onlyIf(() -> Utils.isSimulation()))).withTimeout(Seconds.of(12)); //20-8 seconds climb
    }
}
