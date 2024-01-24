package frc.robot.commands.drive;

import static frc.robot.Constants.LIMELIGHT_NAME;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import frc.robot.commands.shooter.ShootCommand;
import frc.robot.subsystems.DriveTrain;
import frc.robot.util.LimelightHelpers;

public class ShooterDriveCommand extends Command{
    private final DriveTrain driveTrain;
    private final ShootCommand shootCommand;

    private boolean finished = false;
    private double DEADBAND_VALUE = 0.05;

    public ShooterDriveCommand(DriveTrain driveTrain, ShootCommand shootCommand){
        this.driveTrain = driveTrain;
        this.shootCommand = shootCommand;
        this.addRequirements(driveTrain);
    }

    @Override
    public void execute(){
        if(LimelightHelpers.getTV(LIMELIGHT_NAME)){
            if(MathUtil.applyDeadband(LimelightHelpers.getTX(LIMELIGHT_NAME), DEADBAND_VALUE)==0)
                finished = true;
            else if (LimelightHelpers.getTX(LIMELIGHT_NAME) < 0)
                driveTrain.arcadeDrive(DriveTrain.MAX_SPEED, -DriveTrain.MAX_SPEED);
            else if (LimelightHelpers.getTX(LIMELIGHT_NAME) > 0)
                driveTrain.arcadeDrive(DriveTrain.MAX_SPEED, DriveTrain.MAX_SPEED);
            
        }
    }

    @Override
    public boolean isFinished(){
        if(finished)
            CommandScheduler.getInstance().schedule(shootCommand);
        return finished;
    }
}
