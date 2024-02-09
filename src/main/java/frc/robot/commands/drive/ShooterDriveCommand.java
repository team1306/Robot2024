package frc.robot.commands.drive;

import static frc.robot.Constants.LIMELIGHT_NAME;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import frc.robot.commands.shooter.ShootCommand;
import frc.robot.subsystems.DriveTrain;
import frc.robot.util.LimelightHelpers;
import frc.robot.util.MotorUtil;
import frc.robot.util.Utilities;

public class ShooterDriveCommand extends Command{

    private final DriveTrain driveTrain;
    private final ShootCommand shootCommand;
    private final PIDController rotationController;

    private boolean finished = false;
    private double deadbandValue = 0.05;
    public static double P = 1, I = 0, D = 0.1;
    public ShooterDriveCommand(DriveTrain driveTrain, ShootCommand shootCommand){
        this.driveTrain = driveTrain;
        this.shootCommand = shootCommand;
        this.addRequirements(driveTrain);
        this.rotationController = new PIDController(P, I, D);
        SmartDashboard.putNumber("Shooter Auto Deadband", deadbandValue);
    }

    @Override
    public void execute(){
        rotationController.setPID(P, I, D);
        deadbandValue = SmartDashboard.getNumber("Shooter Auto Deadband", 1);
        Pose2d botPose = LimelightHelpers.getBotPose2d(LIMELIGHT_NAME);
        Translation2d targetPos = Utilities.getSpeaker().minus(botPose.getTranslation());
        Rotation2d angle = Rotation2d.fromRadians(Math.atan2(targetPos.getY(), targetPos.getX()));
        Rotation2d robotAngle = botPose.getRotation();
        final double delta = MathUtil.applyDeadband((angle.minus(robotAngle).plus(Rotation2d.fromDegrees(Utilities.isRedAlliance() ? 180 : 0))).getDegrees(), deadbandValue);
        if (delta == 0)
            finished = true;
        else {
            driveTrain.arcadeDrive(0, MotorUtil.clampPercent(rotationController.calculate(delta)));
        }
    }

    @Override
    public boolean isFinished(){
        if(finished)
            CommandScheduler.getInstance().schedule(shootCommand);
        return finished;
    }
}
