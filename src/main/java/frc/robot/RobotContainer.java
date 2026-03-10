// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import static edu.wpi.first.units.Units.MetersPerSecond;

import dev.doglog.DogLog;
import dev.doglog.DogLogOptions;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.Auto.Auto;
import frc.robot.Climber.Climber;
import frc.robot.Drivetrain.Constants;
import frc.robot.Drivetrain.Drivetrain;
import frc.robot.Intake.Intake;
import frc.robot.Shooter.Shooter;
import frc.robot.Spindexer.Spindexer;
import frc.utils.GameData;

public class RobotContainer {
  public Drivetrain drivetrain = Drivetrain.getInstance();
  public Intake intake = Intake.getInstance();
  public Shooter shooter = Shooter.getInstance();
  public Spindexer spindexer = Spindexer.getInstance();
  public Climber climber = Climber.getInstance();
  public GameData game = GameData.getInstance();
  public CommandXboxController MainController = new CommandXboxController(0);
  public CommandXboxController SecondController = new CommandXboxController(1);

  public RobotContainer() {
    drivetrain.setDefaultCommand(drivetrain.drive(
      () -> Constants.MaxVelocity.times(MainController.getLeftY()).times(4.0/5), 
      () -> Constants.MaxVelocity.times(MainController.getLeftX()).times(4.0/5), 
      () -> Constants.MaxOmega
      .times(-1.0).times(Math.abs(MainController.getRightX()) > 0 ? MainController.getRightX() : SecondController.getRightX())));

    intake.setDefaultCommand(intake.setRollVelocity(MetersPerSecond.of(0.5)));
    shooter.setDefaultCommand(shooter.setState(() -> new SwerveModuleState(2, shooter.getState().angle)));
    
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
    // MainController.b().toggleOnTrue(shooter.shoot());
    MainController.b().toggleOnTrue(shooter.setState(new SwerveModuleState(10, Rotation2d.fromDegrees(10))));
    MainController.a().toggleOnTrue(intake.intake(true)); 
    // MainController.y().whileTrue(climber.climb(0.5));
    // MainController.x().whileTrue(climber.climb(-0.5));
    MainController.x().onTrue(shooter.resetPitch());
    MainController.leftBumper().onTrue(intake.moveIntake(true));
    MainController.rightBumper().onTrue(intake.moveIntake(false));
    //TODO: pitch offseting TBD
    //TODO: shoot offseting TBD
    MainController.povUp().toggleOnTrue(drivetrain.faceLock());
    MainController.povDown().toggleOnTrue(drivetrain.robotCentric());
    MainController.povLeft().onTrue(drivetrain.updateVisionPose());
    MainController.povRight().onTrue(intake.CalibrateIntake());
    MainController.back().onTrue(drivetrain.fieldReset());
    MainController.start().onTrue(drivetrain.resetHeading());
    //------------------------------------------------------------------
    SecondController.leftBumper().onTrue(intake.moveIntake(true));
    SecondController.rightBumper().onTrue(intake.moveIntake(false));

    // DrivetrainController.start().onTrue(drivetrain.resetHeading());
    // DrivetrainController.back().onTrue(drivetrain.fieldReset());
    // DrivetrainController.povDown().onTrue(drivetrain.updateVisionPose());
    
    // IntakeController.rightBumper().onTrue(intake.moveIntake(false));
    // IntakeController.leftBumper().onTrue(intake.moveIntake(true));
    // IntakeController.start().onTrue(intake.CalibrateIntake());
    // IntakeController.povDown().toggleOnTrue(shooter.setState(new SwerveModuleState(35, new Rotation2d(Degrees.of(15.5)))));
    
    // new Trigger(() -> DrivetrainController.povUp().getAsBoolean() && IntakeController.povUp().getAsBoolean())
    //   .onTrue(drivetrain.updateVisionPose());

    // new Trigger(() -> game.predictRobotState().isPresent() ? game.predictRobotState().map(state -> state.isNear(game.getRobotState())).orElse(false) : false)
    //     .and(() -> drivetrain.inZone())
    //     .and(() -> game.isHubActive())
    //     .whileTrue(Commands.runEnd(() -> DrivetrainController.setRumble(RumbleType.kBothRumble, 0.8), () -> DrivetrainController.setRumble(RumbleType.kBothRumble, 0)));
  }

  public Command getAutonomousCommand() {
    return Auto.getAutoCommand().beforeStarting(drivetrain.updateVisionPose());
  }
}
