package frc.robot.commands.drive;

import java.util.function.DoubleSupplier;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.DriveTrain;

public class TeleopDriveCommand extends Command{

    private final XboxController controller = new XboxController(1);

    DriveTrain driveTrain;
    private DoubleSupplier forwardSupplier;
    private DoubleSupplier backwardSupplier;
 
    private double DEADBAND_VALUE = 0.05;

    public TeleopDriveCommand(DriveTrain driveTrain, DoubleSupplier forwardSupplier){
        this.driveTrain = driveTrain;
        this.addRequirements(driveTrain);
        SmartDashboard.putNumber("Teleop Drive Deadband", 0.00);
        this.forwardSupplier = forwardSupplier;
        this.backwardSupplier = backwardSupplier;
    }  

    @Override
    public void execute() {

        DEADBAND_VALUE = SmartDashboard.getNumber("Teleop Drive Deadband", 0.00);
        final double forward = forwardSupplier.getAsDouble();
        final double backward = backwardSupplier.getAsDouble();


        //final double forward = xboxController.getRightTriggerAxis(), backward = xboxController.getLeftTriggerAxis();
        double speed = MathUtil.applyDeadband(forward > backward ? forward : -backward, DEADBAND_VALUE);
        double rotation = MathUtil.applyDeadband(controller.getLeftX(), DEADBAND_VALUE);

        driveTrain.arcadeDrive(speed, rotation);
    }

    @Override
    public boolean isFinished(){
        return false;
    }
}
