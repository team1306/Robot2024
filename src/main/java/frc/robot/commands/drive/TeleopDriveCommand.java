package frc.robot.commands.drive;

import java.util.function.BooleanSupplier;
import java.util.function.DoubleSupplier;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.subsystems.DriveTrain;

public class TeleopDriveCommand extends Command{    

    CommandXboxController controller = new CommandXboxController(1); // Creates a controller on port 1.
    

    DriveTrain driveTrain;
    private DoubleSupplier forwardSupplier;
    private DoubleSupplier backwardSupplier;
    private DoubleSupplier rotationSupplier;
    private BooleanSupplier slowModeSupplier;
 
    private double DEADBAND_VALUE = 0.05;

    public TeleopDriveCommand(DriveTrain driveTrain, DoubleSupplier forwardSupplier, DoubleSupplier backwardSupplier, DoubleSupplier rotationSupplier, BooleanSupplier slowModeSupplier){
        this.driveTrain = driveTrain;
        this.addRequirements(driveTrain);
        SmartDashboard.putNumber("Teleop Drive Deadband", 0.00);
        
        this.forwardSupplier = forwardSupplier;
        this.backwardSupplier = backwardSupplier;
        this.rotationSupplier = rotationSupplier;
        this.slowModeSupplier = slowModeSupplier;
    }  

    @Override
    public void execute() {

        DEADBAND_VALUE = SmartDashboard.getNumber("Teleop Drive Deadband", 0.00);
        final double forward = forwardSupplier.getAsDouble();
        final double backward = backwardSupplier.getAsDouble();


        //final double forward = xboxController.getRightTriggerAxis(), backward = xboxController.getLeftTriggerAxis();
        double speed = MathUtil.applyDeadband(forward > backward ? forward : -backward, DEADBAND_VALUE);
        double rotation = MathUtil.applyDeadband(rotationSupplier.getAsDouble(), DEADBAND_VALUE);

        speed *= slowModeSupplier.getAsBoolean() ? 0.3 : 1;
        rotation *= slowModeSupplier.getAsBoolean() ? 0.3 : 1;

        driveTrain.arcadeDrive(speed, rotation);
    }

    @Override
    public boolean isFinished(){
        return false;
    }
}
