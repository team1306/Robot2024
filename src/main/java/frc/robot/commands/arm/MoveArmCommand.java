package frc.robot.commands.arm;

import java.util.function.DoubleSupplier;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Arm;
import frc.robot.subsystems.DriveTrain;

public class MoveArmCommand extends Command {
    private final Arm arm;
    private static double minAngle = 0;
    private static double maxAngle = 90;

    private final DoubleSupplier rotationSupplier;

    public MoveArmCommand(DriveTrain driveTrain, Arm arm, DoubleSupplier rotationSupplier){
        this.arm = arm;
        this.rotationSupplier = rotationSupplier;
        SmartDashboard.putNumber("Arm Min Angle", 0.00);
        SmartDashboard.putNumber("Arm Max Angle", 90);
        this.addRequirements(arm);
    }

    @Override
    public void execute(){
        minAngle = SmartDashboard.getNumber("Arm Min Angle", minAngle);
        maxAngle = SmartDashboard.getNumber("Arm Max Angle", maxAngle);

        double rotationSpeed = rotationSupplier.getAsDouble();
        rotationSpeed *= (rotationSpeed < minAngle || rotationSpeed > maxAngle ? 0.1 : 1);
        arm.setTargetAngle(arm.getCurrentAngle().plus(new Rotation2d(rotationSpeed)));
    }
}