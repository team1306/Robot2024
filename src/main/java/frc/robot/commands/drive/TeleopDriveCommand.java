package frc.robot.commands.drive;

import java.util.function.DoubleSupplier;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.DriveTrain;

public class TeleopDriveCommand extends Command{    
    
    private final DriveTrain driveTrain;
    private final DoubleSupplier forwardSupplier;
    private final DoubleSupplier backwardSupplier;
    private final DoubleSupplier rotationSupplier;
 
    private double deadbandValue = 0.01;
    private boolean lastIsForward = true;
    public TeleopDriveCommand(DriveTrain driveTrain, DoubleSupplier forwardSupplier, DoubleSupplier backwardSupplier, DoubleSupplier rotationSupplier){
        this.driveTrain = driveTrain;
        this.forwardSupplier = forwardSupplier;
        this.backwardSupplier = backwardSupplier;
        this.rotationSupplier = rotationSupplier;
        this.addRequirements(driveTrain);
        
        SmartDashboard.putNumber("Teleop Drive Deadband", deadbandValue);
    }  

    @Override
    public void execute() {
        deadbandValue = SmartDashboard.getNumber("Teleop Drive Deadband", 0.00);
        
        final double forward = forwardSupplier.getAsDouble(), backward = backwardSupplier.getAsDouble(), rotation = rotationSupplier.getAsDouble();
        final boolean isForward = forward > backward;

        final double driveValue = MathUtil.applyDeadband(isForward ? forward : -backward, deadbandValue);
        driveTrain.arcadeDrive(
            driveValue, 
            MathUtil.applyDeadband(rotation * (((Math.abs(driveValue) > 1e-2) ? isForward : lastIsForward) ? 1 : -1), deadbandValue)
        );
        SmartDashboard.putBoolean("is Forward", (Math.abs(driveValue) > 1e-2) ? isForward : lastIsForward);
        lastIsForward = (Math.abs(driveValue) > 1e-3) ? isForward : lastIsForward;
    }
    @Override
    public boolean isFinished(){
        return false;
    }
}
