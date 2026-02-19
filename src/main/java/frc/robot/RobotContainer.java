// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import java.util.Optional;

import dev.doglog.DogLog;
import dev.doglog.DogLogOptions;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.Auto.Auto;
import frc.robot.Drivetrain.Constants;
import frc.robot.Drivetrain.Drivetrain;
import frc.robot.Intake.Intake;
import frc.robot.Intake.Intake.IntakePosition;
import frc.utils.FieldObjects;

public class RobotContainer {
  public Drivetrain drivetrain = Drivetrain.getInstance();
  public Intake intake = Intake.getInstance();
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
    DrivetrainController.a().onTrue(drivetrain.withHeading(Optional.of(FieldObjects.HUB)).ignoringDisable(true));
    DrivetrainController.b().onTrue(drivetrain.withHeading(Optional.empty()).ignoringDisable(true));
    DrivetrainController.x().onTrue(drivetrain.withHeading(Optional.of(FieldObjects.Alliance)).ignoringDisable(true));
    DrivetrainController.start().onTrue(drivetrain.resetHeading());
    DrivetrainController.back().onTrue(drivetrain.fieldReset());
    DrivetrainController.y().onTrue(intake.resetPose().ignoringDisable(true));
    
    IntakeController.rightBumper().onTrue(intake.moveIntake(IntakePosition.Idle));
    IntakeController.leftBumper().onTrue(intake.moveIntake(IntakePosition.Out));
    IntakeController.leftTrigger().whileTrue(intake.intake(true));
    IntakeController.rightTrigger().whileTrue(intake.intake(false));
    IntakeController.start().onTrue(intake.CalibrateIntake());
  }

  public Command getAutonomousCommand() {
    return Auto.getAutoCommand();
  }
}
