package frc.robot.commands.arm;

import java.util.function.DoubleSupplier;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Arm;

public class MoveArmCommand extends Command {
    public static double peakOutput = 0.3;
    private final Arm arm;
    private double minAngle = -1e+9;
    private double maxAngle = 1e+9;
    private double currentAngle = 0;
    private double targetAngle = 0;
    private final DoubleSupplier rotationSupplier;

    public MoveArmCommand(Arm arm, DoubleSupplier rotationSupplier){
        this.arm = arm;
        this.rotationSupplier = rotationSupplier;
        this.addRequirements(arm);

        SmartDashboard.putNumber("Arm Min Angle", 0.00);
        SmartDashboard.putNumber("current angle", 0);
        SmartDashboard.putNumber("Arm Max Angle", 90);
        SmartDashboard.putNumber("Arm Current Angle", arm.getCurrentAngle().getDegrees());
        SmartDashboard.putNumber("Arm Peak Output", 0.3);
        SmartDashboard.putNumber("Arm Target Angle", 0);
    }

    @Override
    public void initialize(){
        arm.setControlMode(Arm.ControlMode.AUTOMATIC);
    }

    @Override
    public void execute(){
        //targetAngle = SmartDashboard.getNumber("Arm Target Angle", targetAngle);
        targetAngle = MathUtil.clamp(Math.max(0, targetAngle + (rotationSupplier.getAsDouble() * -0.12)), minAngle, maxAngle);
        SmartDashboard.putNumber("Arm Target Angle", targetAngle); // to not overwrite value
        SmartDashboard.putNumber("Arm Current Angle", arm.getCurrentAngle().getDegrees());
        minAngle = SmartDashboard.getNumber("Arm Min Angle", minAngle);
        maxAngle = SmartDashboard.getNumber("Arm Max Angle", maxAngle);
        peakOutput = SmartDashboard.getNumber("Arm Peak Output", peakOutput);
        arm.setTargetAngle(Rotation2d.fromDegrees(targetAngle));
    }

    public void reset() {
        targetAngle = arm.getCurrentAngle().getDegrees();
    }
}