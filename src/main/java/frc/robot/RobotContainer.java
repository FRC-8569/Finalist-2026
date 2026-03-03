// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import static edu.wpi.first.units.Units.Degrees;
import static edu.wpi.first.units.Units.MetersPerSecond;
import dev.doglog.DogLog;
import dev.doglog.DogLogOptions;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.Auto.Auto;
import frc.robot.Drivetrain.Constants;
import frc.robot.Drivetrain.Drivetrain;
import frc.robot.Intake.Intake;
import frc.robot.Shooter.Shooter;

public class RobotContainer {
  public Drivetrain drivetrain = Drivetrain.getInstance();
  public Intake intake = Intake.getInstance();
  public Shooter shooter = Shooter.getInstance();
  public CommandXboxController DrivetrainController = new CommandXboxController(0);
  public CommandXboxController IntakeController = new CommandXboxController(1);

  public RobotContainer() {
    drivetrain.setDefaultCommand(drivetrain.drive(
      () -> Constants.MaxVelocity.times(DrivetrainController.getLeftY()).times(Constants.DriveVelocity.div(Constants.MaxVelocity)), 
      () -> Constants.MaxVelocity.times(DrivetrainController.getLeftX()).times(Constants.DriveVelocity.div(Constants.MaxVelocity)), 
      () -> Constants.MaxOmega.times(-DrivetrainController.getRightX())));


   configureBindings();


    DogLog.setOptions(new DogLogOptions()
      .withCaptureConsole(true)
      .withCaptureDs(true)
      .withCaptureNt(true)
      .withNtPublish(true)
      .withLogExtras(true));
    DogLog.setEnabled(true);
  }


  private void configureBindings() {
    DrivetrainController.start().onTrue(drivetrain.resetHeading());
    DrivetrainController.back().onTrue(drivetrain.fieldReset());
    DrivetrainController.povDown().onTrue(drivetrain.updateVisionPose());
    
    IntakeController.rightBumper().onTrue(intake.moveIntake(false));
    IntakeController.leftBumper().onTrue(intake.moveIntake(true));
    IntakeController.leftTrigger().whileTrue(intake.intake(true));
    IntakeController.rightTrigger().whileTrue(intake.intake(false));
    IntakeController.b().onTrue(shooter.setShooterState(MetersPerSecond.of(2))); 
    IntakeController.a().onTrue(shooter.stopShooter());
    IntakeController.y().whileTrue(shooter.setState(new SwerveModuleState(10, new Rotation2d(Degrees.of(10)))));
    IntakeController.x().onTrue(shooter.setShooterState(MetersPerSecond.zero()));
    IntakeController.start().onTrue(intake.CalibrateIntake());
    IntakeController.povDown().onTrue(shooter.resetPitch());

    new Trigger(() -> DrivetrainController.povUp().getAsBoolean() && IntakeController.povUp().getAsBoolean())
      .onTrue(Commands.idle());
  }

  public Command getAutonomousCommand() {
    return Auto.getAutoCommand().beforeStarting(drivetrain.updateVisionPose());
  }
}
