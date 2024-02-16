package frc.robot.commands.arm;

import java.util.function.DoubleSupplier;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Arm;

public class MoveArmCommand extends Command {

    private final Arm arm;
    private double minAngle = -1e+9;
    private double maxAngle = 1e+9;

    private final DoubleSupplier rotationSupplier;

    public MoveArmCommand(Arm arm, DoubleSupplier rotationSupplier){
        this.arm = arm;
        this.rotationSupplier = rotationSupplier;
        this.addRequirements(arm);

        SmartDashboard.putNumber("Arm Min Angle", 0.00);
        SmartDashboard.putNumber("Arm Max Angle", 90);
    }

    @Override
    public void initialize(){
        arm.setControlMode(Arm.ControlMode.MANUAL);
    }

    @Override
    public void execute(){
        minAngle = SmartDashboard.getNumber("Arm Min Angle", minAngle);
        maxAngle = SmartDashboard.getNumber("Arm Max Angle", maxAngle);

        final double armAngle = arm.getCurrentAngle().getDegrees();
        // arm.setManualPower(armAngle < minAngle || armAngle > maxAngle ? 0 : rotationSupplier.getAsDouble());
        arm.setManualPower(rotationSupplier.getAsDouble() * .1);
    }
}