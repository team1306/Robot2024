package frc.robot.auto;

import edu.wpi.first.wpilibj2.command.*;
import frc.robot.commands.arm.MoveArmToSetpointCommand;
import frc.robot.commands.intake.IntakeIndexCommand;
import frc.robot.commands.shooter.ToggleShooterCommand;
import frc.robot.subsystems.Arm;
import frc.robot.subsystems.DriveTrain;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.Shooter;

public class MoveOutMid extends ParallelCommandGroup {
    MoveOutMid(double leftSpeed, double rightSpeed, double time, DriveTrain driveTrain, Shooter shooter, Arm arm, Intake intake, double secondMoveTime) {
        System.out.println("Running Auto");
        final ToggleShooterCommand shooterCommand = new ToggleShooterCommand(() -> 1, shooter);
        addCommands( //all commands run at once
            new InstantCommand(driveTrain::setPoseToVisionPosition),
            shooterCommand, //turns on shooter
            new SequentialCommandGroup( //following commands run in sequence
                new MoveArmToSetpointCommand(arm, Arm.SetpointOptions.SHOOT_CLOSE, () -> true), //aim
                new WaitCommand(1.5),
                new IntakeIndexCommand(intake), //fire
                new InstantCommand(shooterCommand::stop), //turn off shooter
                new MoveArmToSetpointCommand(arm, Arm.SetpointOptions.DOWN, () -> true), //arm down
                new ParallelDeadlineGroup(new WaitCommand(time), //drive out
                    driveTrain.driveBySetpointPercentagesCommand(leftSpeed, rightSpeed)
                ),
                new ParallelDeadlineGroup(new WaitCommand(secondMoveTime), //drive out
                    driveTrain.driveBySetpointPercentagesCommand(Math.max(leftSpeed, rightSpeed), Math.max(leftSpeed, rightSpeed))
                )
            )
        );
    }

    public MoveOutMid(DriveTrain driveTrain, Shooter shooter, Arm arm, Intake intake) {
        this(0.21, 0.20, 1.5, driveTrain, shooter, arm, intake, 1);
    }

    @Override
    public String getName() {
        return "Move Out Mid";
    }
}
