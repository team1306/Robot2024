package frc.robot.commands.drive;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.DriveTrain;

public class TeleopDriveCommand extends Command{

    DriveTrain driveTrain;
    XboxController xboxController;

    private double DEADBAND_VALUE = 0.05;

    public TeleopDriveCommand(DriveTrain driveTrain, XboxController xboxController){
        this.driveTrain = driveTrain;
        this.xboxController = xboxController;
        this.addRequirements(driveTrain);
        SmartDashboard.putNumber("Teleop Drive Deadband", 0.00);

    }  

    @Override
    public void execute(){

        DEADBAND_VALUE = SmartDashboard.getNumber("Teleop Drive Deadband", 0.00);

        final double forward = xboxController.getRightTriggerAxis(), backward = xboxController.getLeftTriggerAxis();
        double speed = MathUtil.applyDeadband(forward > backward ? forward : -backward, DEADBAND_VALUE);
        double rotation = MathUtil.applyDeadband(xboxController.getLeftX(), DEADBAND_VALUE);
        speed *= xboxController.getAButton() ? 0.3 : 1;
        rotation *= xboxController.getAButton() ? 0.3 : 1;

        driveTrain.arcadeDrive(speed, rotation);
    }

    @Override
    public boolean isFinished(){
        return false;
    }
}
