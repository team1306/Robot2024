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
import frc.robot.util.Utilities;

public class ShooterDriveCommand extends Command {

    private final DriveTrain driveTrain;
    private PIDController rotationController;

    public static double kP = 0.0185, kI = 0, kD = 2.2e-3;
    public static final double TOLERANCE_DEGREES = 1;

    public double delta = Double.POSITIVE_INFINITY;

    public ShooterDriveCommand(DriveTrain driveTrain){
        this.driveTrain = driveTrain;
        this.addRequirements(driveTrain);

        DashboardGetter.addGetDoubleData("Shooter Drive kP", kP, value -> kP = value);
        DashboardGetter.addGetDoubleData("Shooter Drive kI", kI, value -> kI = value);
        DashboardGetter.addGetDoubleData("Shooter Drive kD", kD, value -> kD = value);
    }

    /**
     * called every time command is scheduled to ensure that old values do not mess anything up
     */
    @Override
    public void initialize(){
        this.rotationController = new PIDController(kP, kI, kD);
        rotationController.setTolerance(TOLERANCE_DEGREES);
    }

    @Override
    public void execute() {
        rotationController.setPID(kP, kI, kD);
        Pose2d botPose = driveTrain.getPose();

        Translation2d targetPos = Utilities.getSpeaker().minus(botPose.getTranslation());
        Rotation2d angle = Rotation2d.fromRadians(Math.atan2(targetPos.getY(), targetPos.getX()));
        Rotation2d robotAngle = driveTrain.getRotation();

        delta = angle.minus(robotAngle)
            .plus(Rotation2d.fromDegrees(Utilities.isRedAlliance() ? 0 : 180))
            .minus(Rotation2d.fromDegrees(4))
            .getDegrees();

        final double outputPower = MathUtil.clamp(rotationController.calculate(delta), -0.5, 0.5);

        SmartDashboard.putNumber("Delta Drive Angle", delta);
        SmartDashboard.putNumber("Drive PID output", outputPower);

        driveTrain.arcadeDrive(0, outputPower);   
    }

    @Override
    public boolean isFinished() {            
        return Math.abs(delta) < TOLERANCE_DEGREES * 1.5;
    }
}
