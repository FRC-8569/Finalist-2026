// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import static edu.wpi.first.units.Units.Rotation;
import static edu.wpi.first.units.Units.Rotations;

import com.ctre.phoenix6.swerve.SwerveRequest;
import com.ctre.phoenix6.swerve.SwerveModule.SteerRequestType;

import dev.doglog.DogLog;
import dev.doglog.DogLogOptions;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.Drivetrain.Constants;
import frc.robot.Drivetrain.Drivetrain;

public class RobotContainer {
  public Drivetrain drivetrain = Drivetrain.getInstance();
  public XboxController controller = new XboxController(0);

  public RobotContainer() {
    drivetrain.setDefaultCommand(drivetrain.drive(
      () -> Constants.MaxVelocity.times(-controller.getLeftY()).times(Constants.DriveVelocity.div(Constants.MaxVelocity)), 
      () -> Constants.MaxVelocity.times(controller.getLeftX()).times(Constants.DriveVelocity.div(Constants.MaxVelocity)), 
      () -> Constants.MaxOmega.times(controller.getRightX())));
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
  }

  public Command getAutonomousCommand() {
    return drivetrain.runOnce(() -> drivetrain.setControl(new SwerveRequest.PointWheelsAt().withModuleDirection(Rotation2d.kZero).withSteerRequestType(SteerRequestType.MotionMagicExpo)));
  }
}
