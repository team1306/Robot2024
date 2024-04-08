package frc.robot.auto;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj2.command.*;
import frc.robot.commands.arm.MoveArmToSetpointCommand;
import frc.robot.commands.intake.IntakeIndexCommand;
import frc.robot.commands.shooter.ToggleShooterCommand;
import frc.robot.subsystems.Arm;
import frc.robot.subsystems.DriveTrain;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.Shooter;

public class MoveOutMid extends SequentialCommandGroup {
    MoveOutMid(double leftSpeed, double rightSpeed, double time, DriveTrain driveTrain, Shooter shooter, Arm arm, Intake intake, double secondMoveTime) {
        System.out.println("Running Auto");
        addCommands( //all commands run at once
            new InstantCommand(() -> {shooter.setTargetSpeed(.79);driveTrain.setSideVoltages(0, 0);}), //spin up shooter
            arm.getPitchControlCommand(driveTrain),
            new WaitUntilCommand(() -> shooter.getTopRPM() > 4300 && arm.atSetpoint()),
            new IntakeIndexCommand(intake), //fire
            new InstantCommand(() -> shooter.setTargetSpeed(0)),
            new InstantCommand(() -> arm.setTargetAngle(Rotation2d.fromDegrees(0))),
            new ParallelDeadlineGroup(new WaitCommand(time), //drive out
                driveTrain.driveBySetpointPercentagesCommand(leftSpeed, rightSpeed)
            ),
            new ParallelDeadlineGroup(new WaitCommand(secondMoveTime), //drive out
                driveTrain.driveBySetpointPercentagesCommand(Math.max(leftSpeed, rightSpeed), Math.max(leftSpeed, rightSpeed))
            )
        );
    }

    public MoveOutMid(DriveTrain driveTrain, Shooter shooter, Arm arm, Intake intake) {
        this(0.3, 0.15, 1, driveTrain, shooter, arm, intake, 1.6);
    }

    @Override
    public String getName() {
        return "Move Out Mid";
    }
}
