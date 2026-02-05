package frc.robot.Drivetrain;

import static edu.wpi.first.units.Units.Centimeters;
import static edu.wpi.first.units.Units.Degree;
import static edu.wpi.first.units.Units.Degrees;
import static edu.wpi.first.units.Units.Kilograms;
import static edu.wpi.first.units.Units.Milliseconds;
import static edu.wpi.first.units.Units.RotationsPerSecond;
import java.util.Optional;
import java.util.function.Supplier;
import java.util.stream.IntStream;
import org.photonvision.PhotonUtils;

import com.ctre.phoenix6.Utils;
import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.swerve.SwerveModule.DriveRequestType;
import com.ctre.phoenix6.swerve.SwerveModule.SteerRequestType;

import dev.doglog.DogLog;

import com.ctre.phoenix6.swerve.SwerveDrivetrain;
import com.ctre.phoenix6.swerve.SwerveDrivetrainConstants;
import com.ctre.phoenix6.swerve.SwerveModuleConstants;
import com.ctre.phoenix6.swerve.SwerveRequest;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.LinearVelocity;
import edu.wpi.first.units.measure.Time;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Notifier;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Subsystem;
import frc.robot.Drivetrain.Constants.Modules;
import frc.robot.Vision.Vision;
import frc.utils.FieldObjects;

public class Drivetrain extends SwerveDrivetrain<TalonFX, TalonFX, CANcoder> implements Subsystem{
    public static Drivetrain inst;
    public Time SimLoopTime = Milliseconds.of(1);
    private boolean hasDirectionUpdated = false;
    private Notifier SimNotifier;
    private SwerveRequest.ApplyRobotSpeeds autoDrive;
    public SwerveRequest.FieldCentricFacingAngle manualFacing;
    public SwerveRequest.FieldCentric manualDrive;
    public SimulatedDrive SimulatedInstance;
    private Optional<FieldObjects> objects = Optional.empty();
    public Vision vision;

    private Drivetrain(SwerveDrivetrainConstants constants, SwerveModuleConstants<?,?,?>... modules){
        super(TalonFX::new, TalonFX::new, CANcoder::new, constants, Utils.isSimulation() ? SimulatedDrive.regulateModuleConstantsForSimulation(modules) : modules);
        if(Utils.isSimulation()) simulationInit();

        autoDrive = new SwerveRequest.ApplyRobotSpeeds()
            .withDriveRequestType(DriveRequestType.OpenLoopVoltage)
            .withSteerRequestType(SteerRequestType.Position)
            .withDesaturateWheelSpeeds(true);

        manualDrive = new SwerveRequest.FieldCentric()
            .withDriveRequestType(Utils.isSimulation() ? DriveRequestType.OpenLoopVoltage : DriveRequestType.OpenLoopVoltage)
            .withSteerRequestType(SteerRequestType.Position)
            .withDeadband(Constants.MaxVelocity.times(Constants.Deadband))
            .withRotationalDeadband(RotationsPerSecond.of(0.1))
            .withDesaturateWheelSpeeds(true);

        manualFacing = new SwerveRequest.FieldCentricFacingAngle()
            .withDriveRequestType(Utils.isSimulation() ? DriveRequestType.OpenLoopVoltage : DriveRequestType.OpenLoopVoltage)
            .withSteerRequestType(SteerRequestType.Position)
            .withDeadband(Constants.MaxVelocity.times(Constants.Deadband))
            .withRotationalDeadband(Constants.MaxOmega.times(Constants.Deadband))
            .withHeadingPID(1.7, 0, 0);

        vision = Vision.getInstance();
    }

    public Command withHeading(Optional<FieldObjects> obj){
        return runOnce(() -> objects=obj);
    }//

    public Command drive(Supplier<LinearVelocity> vx, Supplier<LinearVelocity> vy, Supplier<AngularVelocity> omega){
        return setControl(() -> objects.isPresent() ? 
        manualFacing.withVelocityX(vx.get()).withVelocityY(vy.get()).withTargetDirection(switch(objects.get()){
            case HUB -> PhotonUtils.getYawToPose(getState().Pose, new Pose3d(Centimeters.of(465),Centimeters.of(403),Centimeters.of(175), new Rotation3d(Degrees.of(0),Degrees.of(-90),Degrees.of(0))).toPose2d());
            case Alliance -> DriverStation.getAlliance().orElseThrow() == Alliance.Red ? Rotation2d.k180deg : Rotation2d.kZero;
        }): 
        manualDrive.withVelocityX(vx.get()).withVelocityY(vy.get()).withRotationalRate(omega.get()));
    }

    public Command setControl(Supplier<SwerveRequest> req){
        return run(() -> setControl(req.get()));
    }

    @Override
    public Command idle(){
        return setControl(() -> new SwerveRequest.Idle());
    }

    @Override
    public void periodic(){
        if(DriverStation.isDisabled() || !hasDirectionUpdated){
            DriverStation.getAlliance().ifPresent(allianceColor -> {
                setOperatorPerspectiveForward(
                    allianceColor == Alliance.Red
                        ? Rotation2d.k180deg
                        : Rotation2d.kZero
                );
                hasDirectionUpdated = true;
            });
        }

        DogLog.log("Drivetrain/ModuleState", getState().ModuleStates);
        DogLog.log("Drivetrain/ModuleTarget", getState().ModuleTargets);
        DogLog.log("Drivetrain/RobotPose", getState().Pose);
        DogLog.log("Drivetrain/RobotSpeeds", getState().Speeds);
        DogLog.log("Drivetrain/ModuleLocation", getState().ModulePositions);
        DogLog.log("Drivetrain/LockingTarget", objects == null ? "null" : objects.toString());
        DogLog.log("Drivetrain/LockingAngle", manualFacing.TargetDirection);

        
        IntStream.range(0, getModules().length).forEachOrdered((i) -> {
            DogLog.log("Drivetrain/Module/%d/SteerMotorVelocity".formatted(i), getModule(i).getSteerMotor().getVelocity().getValue());
            DogLog.log("Drivetrain/Module/%d/SteerMotorAcceleration".formatted(i), getModule(i).getSteerMotor().getAcceleration().getValue());
            DogLog.log("Drivetrain/Module/%d/SteerError".formatted(i), getModule(i).getTargetState().angle.minus(getModule(i).getCurrentState().angle).getDegrees(),Degrees);
            DogLog.log("Drivetrain/Module/%d/SteerAngle".formatted(i), getModule(i).getEncoder().getAbsolutePosition().getValue().in(Degrees), Degree);
        });
        
        vision.getPose().ifPresent(s -> addVisionMeasurement(s.getFirst(), s.getSecond()));

        objects.ifPresent((object) -> {
            DogLog.log("Drivetrain/FacingError", manualFacing.TargetDirection.minus(getState().Pose.getRotation()));
        });
    }

    public Command resetHeading(){
        return runOnce(() -> resetRotation(DriverStation.getAlliance().orElseThrow() == Alliance.Red ? Rotation2d.k180deg : Rotation2d.kZero));
    }

    @Override
    public void resetPose(Pose2d Pose){
        if(SimulatedInstance != null) SimulatedInstance.mapleSimDrive.setSimulationWorldPose(Pose);
        super.resetPose(Pose);
    }

    @SuppressWarnings("unchecked")
    private void simulationInit(){
        SimulatedInstance = new SimulatedDrive(
            SimLoopTime, 
            Kilograms.of(52), 
            Centimeters.of(80), 
            Centimeters.of(80), 
            DCMotor.getKrakenX60Foc(1), 
            DCMotor.getKrakenX60(1), 
            1.2, 
            getModuleLocations(), 
            getPigeon2(), 
            getModules(), 
            Modules.FrontLeft.constant,
            Modules.FrontRight.constant,
            Modules.BackLeft.constant,
            Modules.BackRight.constant);
        SimNotifier = new Notifier(SimulatedInstance::update);
        SimNotifier.startPeriodic(SimLoopTime);
    }

    public static Drivetrain getInstance(){
        inst = inst == null ? new Drivetrain(Constants.constants,Modules.FrontLeft.constant, Modules.FrontRight.constant, Modules.BackLeft.constant, Modules.BackRight.constant) : inst;
        return inst;
    }
}
