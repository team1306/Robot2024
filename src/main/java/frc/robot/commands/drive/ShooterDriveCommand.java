package frc.robot.commands.drive;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.DriveTrain;
import frc.robot.util.DashboardGetter;
import frc.robot.util.MotorUtil;
import frc.robot.util.Utilities;

public class ShooterDriveCommand extends Command{

    private final DriveTrain driveTrain;
    private final PIDController rotationController;

    private boolean finished = false;
    private double deadbandValue = 0.5;
    public static double kP = 0.025, kI = 0, kD = 0.001;

    public ShooterDriveCommand(DriveTrain driveTrain){
        this.driveTrain = driveTrain;
        this.rotationController = new PIDController(kP, kI, kD);
        this.addRequirements(driveTrain);

        DashboardGetter.addGetDoubleData("Shooter Drive kP", kP, value -> kP = value);
        DashboardGetter.addGetDoubleData("Shooter Drive kI", kI, value -> kI = value);
        DashboardGetter.addGetDoubleData("Shooter Drive kD", kD, value -> kD = value);
        DashboardGetter.addGetDoubleData("Shooter Auto Deadband", deadbandValue, value -> deadbandValue = value);

    }

    @Override
    public void initialize(){
        finished = false;
    }
    @Override
    public void execute(){
        rotationController.setPID(kP, kI, kD);
        Pose2d botPose = Utilities.getRobotPos();
        // double speakerDistance = Utilities.getSpeakerDistance(botPose);

        Translation2d targetPos = Utilities.getSpeaker().minus(botPose.getTranslation());
        Rotation2d angle = Rotation2d.fromRadians(Math.atan2(targetPos.getY(), targetPos.getX()));
        Rotation2d robotAngle = botPose.getRotation();


        final double delta = MathUtil.applyDeadband((
            angle.minus(robotAngle)
            .plus(Rotation2d.fromDegrees(Utilities.isRedAlliance() ? 0 : 180))
            // .plus(Rotation2d.fromDegrees(Units.metersToInches(speakerDistance) * (54.0/7.0)))
            )
            .getDegrees(), deadbandValue);
        
        SmartDashboard.putNumber("Delta Drive Angle", delta);
        SmartDashboard.putNumber("Drive PID output", MotorUtil.clampPercent(rotationController.calculate(delta)) * 0.5);

        if (delta == 0) finished = true;
        else driveTrain.arcadeDrive(0, MotorUtil.clampPercent(rotationController.calculate(delta)) * 0.5);
        
    }

    @Override
    public boolean isFinished(){            
        return finished;
    }
}
