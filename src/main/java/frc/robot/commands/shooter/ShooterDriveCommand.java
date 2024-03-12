package frc.robot.commands.shooter;

import static frc.robot.Constants.INCLUDE_LIMELIGHT;
import static frc.robot.Constants.LIMELIGHT_NAME;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import frc.robot.subsystems.DriveTrain;
import frc.robot.util.LimelightHelpers;
import frc.robot.util.MotorUtil;
import frc.robot.util.Utilities;

public class ShooterDriveCommand extends Command{

    private final DriveTrain driveTrain;
    private final NoteIndexingCommand noteIndexCommand;
    private final ToggleShooterCommand toggleShooterCommand;
    private final PIDController rotationController;

    private boolean finished = false;
    private double deadbandValue = 0.05;
    public static double kP = 1, kI = 0, kD = 0;

    public ShooterDriveCommand(DriveTrain driveTrain, NoteIndexingCommand noteIndexCommand, ToggleShooterCommand toggleShooterCommand){
        this.driveTrain = driveTrain;
        this.noteIndexCommand = noteIndexCommand;
        this.rotationController = new PIDController(kP, kI, kD);
        this.toggleShooterCommand = toggleShooterCommand;
        this.addRequirements(driveTrain);
         
        SmartDashboard.putNumber("Shooter Drive kP", kP);
        SmartDashboard.putNumber("Shooter Drive kI", kI);
        SmartDashboard.putNumber("Shooter Drive kD", kD);
        SmartDashboard.putNumber("Shooter Auto Deadband", deadbandValue);
    }

    @Override
    public void execute(){
        kP = SmartDashboard.getNumber("Shooter Drive kP", 1);
        kI = SmartDashboard.getNumber("Shooter Drive kI", 0);
        kD = SmartDashboard.getNumber("Shooter Drive kD", 0);
        deadbandValue = SmartDashboard.getNumber("Shooter Auto Deadband", 1);

        rotationController.setPID(kP, kI, kD);
        Pose2d botPose = INCLUDE_LIMELIGHT ? LimelightHelpers.getBotPose2d(LIMELIGHT_NAME) : new Pose2d();
        Translation2d targetPos = Utilities.getSpeaker().minus(botPose.getTranslation());
        Rotation2d angle = Rotation2d.fromRadians(Math.atan2(targetPos.getY(), targetPos.getX()));
        Rotation2d robotAngle = botPose.getRotation();

        final double delta = MathUtil.applyDeadband(
            (angle.minus(robotAngle).plus(Rotation2d.fromDegrees(Utilities.isRedAlliance() ? 180 : 0))).getDegrees(), deadbandValue);
        
        if (delta == 0) finished = true;
        else driveTrain.arcadeDrive(0, MotorUtil.clampPercent(rotationController.calculate(delta) * 0.25));
        
    }

    @Override
    public boolean isFinished(){            
        return finished;
    }

    @Override
    public void end(boolean interrupted){
        CommandScheduler.getInstance().schedule(
            noteIndexCommand.alongWith(toggleShooterCommand)
            .withTimeout(NoteIndexingCommand.TIME_AFTER_SHOT_MS + NoteIndexingCommand.TIME_BEFORE_SHOT_MS));
    }
}
