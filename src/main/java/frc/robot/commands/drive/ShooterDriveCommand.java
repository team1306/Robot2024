package frc.robot.commands.drive;

import static frc.robot.Constants.BLUE_SPEAKER;
import static frc.robot.Constants.LIMELIGHT_NAME;
import static frc.robot.Constants.RED_SPEAKER;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import frc.robot.commands.shooter.ShootCommand;
import frc.robot.subsystems.DriveTrain;
import frc.robot.util.LimelightHelpers;
import frc.robot.util.Utilities;

public class ShooterDriveCommand extends Command{

    private final DriveTrain driveTrain;
    private final ShootCommand shootCommand;

    private boolean finished = false;
    private double DEADBAND_VALUE = 0.05;

    public ShooterDriveCommand(DriveTrain driveTrain, ShootCommand shootCommand){
        this.driveTrain = driveTrain;
        this.shootCommand = shootCommand;
        this.addRequirements(driveTrain);

        SmartDashboard.putNumber("Shooter Auto Deadband", DEADBAND_VALUE);
    }

    @Override
    public void execute(){
        DEADBAND_VALUE = SmartDashboard.getNumber("Shooter Auto Deadband", 0.05);

        Pose2d botPose = LimelightHelpers.getBotPose2d(LIMELIGHT_NAME);
        Translation2d targetPos = (Utilities.isRedAlliance() ? RED_SPEAKER : BLUE_SPEAKER).minus(botPose.getTranslation());
        double angle = Math.atan(targetPos.getY()/targetPos.getX());
        double robotAngle = botPose.getRotation().getRadians();
        //TODO Fix bad code (probably doesn't work)
        if (MathUtil.applyDeadband((angle - robotAngle + (Utilities.isRedAlliance() ? 180 : 0)), DEADBAND_VALUE) == 0)
            finished = true;
        else{
            if (angle - robotAngle + (Utilities.isRedAlliance() ? 180 : 0) < 0){
                driveTrain.arcadeDrive(0, -DriveTrain.MAX_SPEED);
            }else{
                driveTrain.arcadeDrive(0, DriveTrain.MAX_SPEED);
            }
        }
    }

    @Override
    public boolean isFinished(){
        if(finished)
            CommandScheduler.getInstance().schedule(shootCommand);
        return finished;
    }
}
