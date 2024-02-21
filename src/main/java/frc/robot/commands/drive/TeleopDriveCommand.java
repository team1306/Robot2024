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
 
    private double DEADBAND_VALUE = 0.05;

    public TeleopDriveCommand(DriveTrain driveTrain, DoubleSupplier forwardSupplier, DoubleSupplier backwardSupplier, DoubleSupplier rotationSupplier){
        this.driveTrain = driveTrain;
        this.forwardSupplier = forwardSupplier;
        this.backwardSupplier = backwardSupplier;
        this.rotationSupplier = rotationSupplier;
        this.addRequirements(driveTrain);
        
        SmartDashboard.putNumber("Teleop Drive Deadband", 0.00);
    }  

    @Override
    public void execute() {
        DEADBAND_VALUE = SmartDashboard.getNumber("Teleop Drive Deadband", 0.00);
        
        final double forward = forwardSupplier.getAsDouble(), backward = backwardSupplier.getAsDouble();
        final boolean isForward = forward > backward;
        double speed = MathUtil.applyDeadband(isForward ? forward : -backward, DEADBAND_VALUE);
        double rotation = MathUtil.applyDeadband(rotationSupplier.getAsDouble() * (isForward ? 1 : -1), DEADBAND_VALUE);

        driveTrain.arcadeDrive(speed, rotation);
    }

    @Override
    public boolean isFinished(){
        return false;
    }
}
