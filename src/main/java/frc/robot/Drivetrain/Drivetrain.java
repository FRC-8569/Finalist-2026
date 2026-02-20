package frc.robot.Drivetrain;

import static edu.wpi.first.units.Units.Milliseconds;

import java.util.Optional;
import java.util.function.Supplier;

import org.photonvision.PhotonUtils;

import com.ctre.phoenix6.CANBus.CANBusStatus;
import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.swerve.SwerveDrivetrain;
import com.ctre.phoenix6.swerve.SwerveDrivetrainConstants;
import com.ctre.phoenix6.swerve.SwerveModuleConstants;
import com.ctre.phoenix6.swerve.SwerveRequest;
import com.ctre.phoenix6.swerve.SwerveModule.DriveRequestType;
import com.ctre.phoenix6.swerve.SwerveModule.SteerRequestType;
import com.ctre.phoenix6.swerve.SwerveRequest.ApplyFieldSpeeds;

import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.LinearVelocity;
import edu.wpi.first.units.measure.Time;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Notifier;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Subsystem;
import frc.robot.Drivetrain.Constants.Modules;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.config.RobotConfig;

public class Drivetrain extends SwerveDrivetrain<TalonFX,TalonFX, CANcoder> implements Subsystem{
    private static Drivetrain inst;
    public Time SimLoopTime = Milliseconds.of(1);
    private boolean hasDirectionUpdated = false;
    private Notifier SimNotifier;
    private SwerveRequest.ApplyFieldSpeeds AutoDrive;
    public SwerveRequest.FieldCentric ManualDrive;
    public SwerveRequest.FieldCentricFacingAngle ManualFacing;

    public Drivetrain(SwerveDrivetrainConstants constants, SwerveModuleConstants<?,?,?>... modules){
        super(TalonFX::new, TalonFX::new, CANcoder::new, constants, modules);

        AutoDrive = new ApplyFieldSpeeds()
            .withDriveRequestType(DriveRequestType.OpenLoopVoltage)
            .withSteerRequestType(SteerRequestType.Position)
            .withDesaturateWheelSpeeds(true);
        ManualDrive = new SwerveRequest.FieldCentric()
            .withDriveRequestType(DriveRequestType.Velocity)
            .withSteerRequestType(SteerRequestType.Position)
            .withDesaturateWheelSpeeds(true);
        ManualFacing = new SwerveRequest.FieldCentricFacingAngle()
            .withDriveRequestType(DriveRequestType.Velocity)
            .withSteerRequestType(SteerRequestType.Position)
            .withDesaturateWheelSpeeds(true);

        AutoInit();

        CANBusStatus status = Constants.bus.getStatus();

    }

    public Command drive(Supplier<LinearVelocity> vx, Supplier<LinearVelocity> vy, Supplier<AngularVelocity> omega){
        return run(() -> setControl(ManualDrive.withVelocityX(vx.get()).withVelocityY(vy.get()).withRotationalRate(omega.get())));
    }

    private void AutoInit(){
        try{
            AutoBuilder.configure(
                () -> getState().Pose, 
                this::resetPose, 
                () -> getState().Speeds, 
                (speeds, ff) -> setControl(AutoDrive.withSpeeds(speeds).withWheelForceFeedforwardsX(ff.robotRelativeForcesX()).withWheelForceFeedforwardsY(ff.robotRelativeForcesY())), 
                Constants.AutoPID, 
                RobotConfig.fromGUISettings(), 
                () -> DriverStation.getAlliance().orElseThrow() == Alliance.Red, 
                this);
        }catch(Exception e){
            DriverStation.reportError(e.getMessage(), e.getStackTrace());
        }
    }

    public static Drivetrain getInstance(){
        inst = inst == null ? new Drivetrain(Constants.constants,Modules.FrontLeft.constant, Modules.FrontRight.constant, Modules.BackLeft.constant, Modules.BackRight.constant) : inst;
        return inst;
    }
}
