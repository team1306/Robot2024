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
        driveTrain.arcadeDrive(Math.pow(forwardSupplier.getAsDouble(), 2), Math.pow(rotationSupplier.getAsDouble(), 2));
    }
    
    @Override
    public boolean isFinished(){
        return false;
    }
}
