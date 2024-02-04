package frc.robot.commands.drive;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.DriveTrain;

public class TeleopDriveCommand extends Command{

    DriveTrain driveTrain;
    XboxController xboxController;

    public TeleopDriveCommand(DriveTrain driveTrain, XboxController xboxController){
        this.driveTrain = driveTrain;
        this.xboxController = xboxController;
        this.addRequirements(driveTrain);
    }  

    @Override
    public void execute(){
        double speed = MathUtil.applyDeadband(xboxController.getLeftY(), 0.05);
        double rotation = MathUtil.applyDeadband(xboxController.getLeftX(), 0.05);

        driveTrain.arcadeDrive(speed, rotation);
    }

    @Override
    public boolean isFinished(){
        return false;
    }
}
