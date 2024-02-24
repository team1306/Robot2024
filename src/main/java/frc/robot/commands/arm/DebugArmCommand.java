package frc.robot.commands.arm;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Arm;

public class DebugArmCommand extends Command {
    public static double peakOutput = 1;
    private final Arm arm;
    private double minAngle = -1e+9;
    private double maxAngle = 1e+9;
    private double targetAngle = 0;

    public DebugArmCommand(Arm arm){
        this.arm = arm;
        this.addRequirements(arm);

        SmartDashboard.putNumber("Arm Min Angle", 0.00);
        SmartDashboard.putNumber("current angle", 0);
        SmartDashboard.putNumber("Arm Max Angle", 90);
        SmartDashboard.putNumber("Arm Current Angle", arm.getCurrentAngle().getDegrees());
        SmartDashboard.putNumber("Arm Peak Output", 1);
        SmartDashboard.putNumber("Arm Target Angle", 0);
    }

    @Override
    public void initialize(){
        arm.setControlMode(Arm.ControlMode.AUTOMATIC);
    }

    @Override
    public void execute(){
        targetAngle = SmartDashboard.getNumber("Arm Target Angle", targetAngle);
        SmartDashboard.putNumber("Arm Target Angle", targetAngle); // to not overwrite value
        SmartDashboard.putNumber("Arm Current Angle", arm.getCurrentAngle().getDegrees());
        minAngle = SmartDashboard.getNumber("Arm Min Angle", minAngle);
        maxAngle = SmartDashboard.getNumber("Arm Max Angle", maxAngle);
        peakOutput = SmartDashboard.getNumber("Arm Peak Output", peakOutput);
        arm.setTargetAngle(Rotation2d.fromDegrees(targetAngle));
    }

    public void reset() {
        final Rotation2d armCurrentAngle = arm.getCurrentAngle();
        arm.setTargetAngle(armCurrentAngle);
        targetAngle = armCurrentAngle.getDegrees();
    }
}