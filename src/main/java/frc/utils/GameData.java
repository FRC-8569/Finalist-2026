package frc.utils;

import static edu.wpi.first.units.Units.MetersPerSecond;

import java.util.Arrays;
import java.util.Optional;

import com.ctre.phoenix6.CANBus;
import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.swerve.SwerveModule;

import dev.doglog.DogLog;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj.Alert;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Alert.AlertType;
import frc.utils.ShootUtils.RobotState;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj2.command.Subsystem;
import frc.robot.Drivetrain.Drivetrain;
import frc.robot.Intake.Intake;
import frc.robot.Shooter.Shooter;
import frc.robot.Spindexer.Spindexer;

public class GameData implements Subsystem{
    public Optional<FieldObjects> CurrentLocking = Optional.empty();
    private static GameData inst;
    private boolean manualCancel = false;
    public Alert CANAlert = new Alert("Robot", "Initial Value, Waiting update", AlertType.kInfo);

    private GameData(){
        register();
        CANAlert.set(true);
    }

    public void withLocking(FieldObjects obj){
        this.CurrentLocking = Optional.of(obj);
    }

    public void clearLocking(){
        resetLocking();
        manualCancel = true;
    }

    private void resetLocking(){
        this.CurrentLocking = Optional.empty();
    }

    public boolean isHubActive(){
        Alliance activeHub = switch(DriverStation.getGameSpecificMessage()){
            case "R" -> Alliance.Red;
            case "B" -> Alliance.Blue;
            default -> null;
        };

        return activeHub == DriverStation.getAlliance().orElseThrow();
    }

    public Optional<RobotState> predictRobotState(FieldObjects obj){
        Pose2d obj2d = new Pose2d(obj.getPose().getX(), obj.getPose().getY(), new Rotation2d(obj.getPose().getRotation().getMeasureZ()));
        obj2d = PoseUtils.getPose(obj2d, DriverStation.getAlliance().orElseThrow(), null);
        return predictRobotState(new Pose3d(obj2d.getX(), obj2d.getY(), obj.getPose().getZ(),new Rotation3d(obj.getPose().getRotation().getMeasureX(), obj.getPose().getRotation().getMeasureY(), obj2d.getRotation().getMeasure())));
    }

    public Optional<RobotState> predictRobotState(Pose3d pose){
        Optional<RobotState> state = ShootUtils.calculateState(new Transform3d(new Pose3d(Drivetrain.getInstance().getState().Pose.rotateBy(Rotation2d.k180deg)), pose));

        return state;
    }

    public RobotState getRobotState(){
        return new RobotState(Drivetrain.getInstance().getState().Pose.getRotation(), MetersPerSecond.of(Shooter.getInstance().getState().speedMetersPerSecond), Shooter.getInstance().getState().angle.getMeasure());
    }

    @Override
    public void periodic(){
        // if(isHubActive() && predictRobotState().isPresent() && !manualCancel && Drivetrain.getInstance().inZone()) CurrentLocking = Optional.of(FieldObjects.HUB);
        predictRobotState();
        if(Arrays.stream(Drivetrain.getInstance().getModules()).<TalonFX>map(SwerveModule::getDriveMotor).map(TalonFX::isConnected).allMatch(s -> s == true) &&
        Arrays.stream(Drivetrain.getInstance().getModules()).<TalonFX>map(SwerveModule::getSteerMotor).map(TalonFX::isConnected).allMatch(s -> s == true) &&
        Arrays.stream(Drivetrain.getInstance().getModules()).<CANcoder>map(SwerveModule::getEncoder).map(CANcoder::isConnected).allMatch(s -> s == true) &&
        Intake.getInstance().TongueMotor.isConnected() && Intake.getInstance().RollMotor.isConnected() && 
        Shooter.getInstance().PitchingMotor.isConnected() && Shooter.getInstance().ShootingMotor.isConnected() &&
        Spindexer.getInstance().SpindexMotor.isConnected() && Spindexer.getInstance().IndexMotor.isConnected()) CANAlert.setText("All device connected, YAY");
        else CANAlert.setText("CANBus missing device, check it");;
    }

    /**
     * This function will return the predicted robot state of shooting to the HUB, for general use, please see {@link #predictRobotState(FieldObjects)} or {@link #predictRobotState(Pose3d)}
     * @return
     */
    public Optional<RobotState> predictRobotState(){
        return predictRobotState(FieldObjects.HUB);
    }

    public static GameData getInstance(){
        inst = inst == null ? new GameData() : inst;
        return inst;
    }
}