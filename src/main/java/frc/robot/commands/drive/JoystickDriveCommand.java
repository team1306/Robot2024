package frc.robot.commands.drive;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.DriveTrain;
import frc.robot.util.DashboardGetter;

import java.util.function.DoubleSupplier;

public class JoystickDriveCommand extends Command{    
    
    private final DriveTrain driveTrain;
    private final DoubleSupplier forwardSupplier;
    private final DoubleSupplier rotationSupplier;
     
    public JoystickDriveCommand(DriveTrain driveTrain, DoubleSupplier leftYSupplier, DoubleSupplier rotationSupplier) {
        this.driveTrain = driveTrain;
        this.forwardSupplier = leftYSupplier;
        this.rotationSupplier = rotationSupplier;
        this.addRequirements(driveTrain);
    }

    @Override
    public void execute() {
        double forward = Math.copySign(Math.pow(forwardSupplier.getAsDouble(), 2), forwardSupplier.getAsDouble());
        double rotation = Math.copySign(Math.pow(rotationSupplier.getAsDouble(), 2), rotationSupplier.getAsDouble());
        driveTrain.arcadeDrive(forward, rotation);
    }
    
    @Override
    public boolean isFinished(){
        return false;
    }
}
