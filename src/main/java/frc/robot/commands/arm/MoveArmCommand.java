package frc.robot.commands.arm;

import java.util.function.DoubleSupplier;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Arm;

public class MoveArmCommand extends Command {
    public static double peakOutput = 1;
    private final Arm arm;
    private double minAngle = -1e+9;
    private double maxAngle = 1e+9;
    public static double speed = 0.42;
    private double targetAngle = 0;
    private final DoubleSupplier rotationSupplier;

    public MoveArmCommand(Arm arm, DoubleSupplier rotationSupplier){
        this.arm = arm;
        this.rotationSupplier = rotationSupplier;
        this.addRequirements(arm);

        SmartDashboard.putNumber("Arm Min Angle", 0.00);
        SmartDashboard.putNumber("current angle", 0);
        SmartDashboard.putNumber("Arm Max Angle", 90);
        SmartDashboard.putNumber("Arm Peak Output", 1);
        SmartDashboard.putNumber("Arm Target Angle", 0);
        SmartDashboard.putNumber("Arm Speed", speed);
    }

    @Override
    public void initialize(){
        arm.setControlMode(Arm.ControlMode.AUTOMATIC);
    }

    @Override
    public void execute(){
        speed = SmartDashboard.getNumber("Arm Speed", speed);
        targetAngle = Math.max(0, targetAngle + (rotationSupplier.getAsDouble() * -1 * speed));
        minAngle = SmartDashboard.getNumber("Arm Min Angle", minAngle);
        maxAngle = SmartDashboard.getNumber("Arm Max Angle", maxAngle);
        peakOutput = SmartDashboard.getNumber("Arm Peak Output", peakOutput);
        SmartDashboard.putNumber("Arm Target Angle", targetAngle);
        arm.setTargetAngle(Rotation2d.fromDegrees(targetAngle));
    }

    public void reset() {
        final Rotation2d armCurrentAngle = arm.getCurrentAngle();
        arm.setTargetAngle(armCurrentAngle);
        targetAngle = armCurrentAngle.getDegrees();
    }
}