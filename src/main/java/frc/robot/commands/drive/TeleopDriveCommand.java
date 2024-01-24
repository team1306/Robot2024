package frc.robot.commands.drive;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.DriveTrain;

public class TeleopDriveCommand extends Command{

    DriveTrain driveTrain;
    Joystick leftJoystick;

    public TeleopDriveCommand(DriveTrain driveTrain, Joystick leftJoystick){
        this.driveTrain = driveTrain;
        this.leftJoystick = leftJoystick;
        this.addRequirements(driveTrain);
    }  

    @Override
    public void execute(){
        double speed = MathUtil.applyDeadband(leftJoystick.getX(), 0.05);
        double rotation = MathUtil.applyDeadband(leftJoystick.getY(), 0.05);

        driveTrain.arcadeDrive(speed, rotation);
    }
}
