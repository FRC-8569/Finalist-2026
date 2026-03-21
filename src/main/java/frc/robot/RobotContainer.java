// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import static edu.wpi.first.units.Units.MetersPerSecond;

import java.util.List;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.auto.NamedCommands;
import com.pathplanner.lib.commands.PathPlannerAuto;

import dev.doglog.DogLog;
import dev.doglog.DogLogOptions;
import edu.wpi.first.math.Pair;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.util.sendable.SendableBuilder;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.Auto.CompoundCommand;
import frc.robot.Auto.PathPlannerOnlyCommands;
import frc.robot.Drivetrain.Constants;
import frc.robot.Drivetrain.Drivetrain;
import frc.robot.Intake.Intake;
import frc.robot.Shooter.Shooter;
import frc.robot.Spindexer.Spindexer;
import frc.utils.Tools;
import frc.utils.Tools.RobotState;
import frc.utils.Tools.ShootPreset;

public class RobotContainer {
  public Drivetrain drivetrain = Drivetrain.getInstance();
  public Intake intake = Intake.getInstance();
  public Shooter shooter = Shooter.getInstance();
  public Spindexer spindexer = Spindexer.getInstance();
  // public Climber climber = Climber.getInstance();
  public CommandXboxController MainController = new CommandXboxController(0);
  public CommandXboxController SecondController = new CommandXboxController(1);
  public SendableChooser<ShootPreset> PresetShooter = new SendableChooser<ShootPreset>();
  public boolean manualShoot = false;
  private SendableChooser<Command> AutoChooser = AutoBuilder.buildAutoChooser();

  public RobotContainer() {
    NamedCommands.registerCommands(List.of(
      Pair.of("ShortWhile", PathPlannerOnlyCommands.shootShortWhile()),
      Pair.of("LongWhile", PathPlannerOnlyCommands.shootLongWhile())
    ));

    drivetrain.setDefaultCommand(drivetrain.drive(
      () -> Constants.MaxVelocity.times(MainController.getLeftY()).times(4.0/5), 
      () -> Constants.MaxVelocity.times(MainController.getLeftX()).times(4.0/5), 
      () -> Constants.MaxOmega.times(-1.0).times(Math.abs(MainController.getRightX()) > 0 ? MainController.getRightX() : SecondController.getRawAxis(3))));

    shooter.setDefaultCommand(shooter.setState(() -> new SwerveModuleState(2, Rotation2d.fromDegrees(90).minus(shooter.getState().angle))));
    intake.setDefaultCommand(intake.intake(true));
    configureBindings();


    DogLog.setOptions(new DogLogOptions()
      .withCaptureConsole(true)
      .withCaptureDs(true)
      .withCaptureNt(true)
      .withNtPublish(true)
      .withLogExtras(true));
    DogLog.setEnabled(true);
    SmartDashboard.putData("AutonomousChooser", AutoChooser);
  }


  private void configureBindings() {
    MainController.a().toggleOnTrue(intake.setRollVelocity()); 
    MainController.b().toggleOnTrue(
      CompoundCommand.shoot(() -> PresetShooter.getSelected()).until(() -> (Math.abs(MainController.getLeftX())+Math.abs(MainController.getLeftY()+Math.abs(MainController.getRightX())) > 0.1))
      .onlyIf(() -> drivetrain.inZone())); 
    // MainController.b().toggleOnTrue(shooter.setState(RobotState.getNeutralState()).until(() -> (Math.abs(MainController.getLeftX())+Math.abs(MainController.getLeftY()+Math.abs(MainController.getRightX())) > 0.1))
    // .alongWith(Commands.runOnce(() -> DogLog.log("Driver/Shooter/manualControl", false))));
    MainController.b().toggleOnTrue(shooter.setState(new SwerveModuleState(18, Rotation2d.fromDegrees(90-27))));
    MainController.x().onTrue(intake.CalibrateIntake());
    MainController.y().onTrue(drivetrain.updateVisionPose());
    MainController.leftBumper().onTrue(intake.moveIntake(true));
    MainController.rightBumper().onTrue(intake.moveIntake(false));
    // MainController.leftTrigger().whileTrue(climber.climb(0.5));
    // MainController.rightTrigger().whileTrue(climber.climb(-0.5));

    MainController.leftStick().toggleOnTrue(drivetrain.faceLock());
    MainController.povDown().onTrue(shooter.calibrateShooter().onlyIf(() -> DriverStation.isDisabled()));
    MainController.povUp().onTrue(drivetrain.robotCentric().onlyIf(() -> DriverStation.isEnabled()));
    MainController.povLeft().toggleOnTrue(CompoundCommand.shoot(Tools.LeftBump).until(() -> (Math.abs(MainController.getLeftX())+Math.abs(MainController.getLeftY()+Math.abs(MainController.getRightX())) > 0.1)));
    MainController.povRight().toggleOnTrue(CompoundCommand.shoot(Tools.RightBump).until(() ->  (Math.abs(MainController.getLeftX())+Math.abs(MainController.getLeftY()+Math.abs(MainController.getRightX())) > 0.1)));
    MainController.back().onTrue(drivetrain.fieldReset());
    MainController.start().onTrue(drivetrain.resetHeading());
    // new Trigger(() -> MainController.getLeftTriggerAxis()+MainController.getRightTriggerAxis() > 0.1)
    MainController.rightStick()
      .whileTrue(shooter.setState(() -> new SwerveModuleState(MetersPerSecond.of(5), Rotation2d.fromDegrees(90-25.6))));
    //------------------------------------------------------------------

    // SecondController.leftBumper().onTrue(intake.moveIntake(true));
    // SecondController.rightBumper().onTrue(intake.moveIntake(false));
    // SecondController.x().onTrue(drivetrain.updateVisionPose());
    // SecondController.y().onTrue(intake.CalibrateIntake());
    
    SecondController.leftBumper().onTrue(intake.moveIntake(true));
    SecondController.rightBumper().onTrue(intake.moveIntake(false));
    SecondController.x().onTrue(intake.CalibrateIntake());
    SecondController.y().onTrue(drivetrain.updateVisionPose());
    SecondController.a().onTrue(intake.setRollVelocity());
    SecondController.b().toggleOnTrue(shooter.setState(new SwerveModuleState(18, Rotation2d.fromDegrees(90-27))));
    SecondController.povLeft().or(SecondController.povRight()).toggleOnTrue(shooter.setState(Tools.RightBump.state()));
  }

  public Command getAutonomousCommand() {
    return AutoChooser.getSelected();
  }
}
