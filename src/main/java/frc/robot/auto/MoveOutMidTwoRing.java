package frc.robot.auto;

import edu.wpi.first.wpilibj2.command.*;
import frc.robot.commands.arm.MoveArmToSetpointCommand;
import frc.robot.commands.intake.IntakeIndexCommand;
import frc.robot.commands.shooter.ToggleShooterCommand;
import frc.robot.subsystems.Arm;
import frc.robot.subsystems.DriveTrain;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.Shooter;

import static frc.robot.commands.intake.IntakeDriverCommand.INTAKE_SPEED;

import com.pathplanner.lib.path.PathPlannerPath;


public class MoveOutMidTwoRing extends ParallelCommandGroup {
    public MoveOutMidTwoRing(DriveTrain driveTrain, Shooter shooter, Arm arm, Intake intake) {
        driveTrain.resetPose(PathPlannerPath.fromPathFile("Start-2 to Shoot-Top").getStartingDifferentialPose());
        System.out.println("Running Auto");
        final ToggleShooterCommand shooterCommand = new ToggleShooterCommand(() -> .76, shooter);
        addCommands( //all commands run at once
            shooterCommand, //turns on shooter
            new SequentialCommandGroup( //following commands run in sequence
                new MoveArmToSetpointCommand(arm, Arm.SetpointOptions.SHOOT_CLOSE, () -> true), //aim
                new WaitCommand(1.5),
                new IntakeIndexCommand(intake), //fire
                new InstantCommand(() -> intake.setTargetSpeed(INTAKE_SPEED)),
                new MoveArmToSetpointCommand(arm, Arm.SetpointOptions.DOWN, () -> true), //arm down
                //turn on intake
                new ParallelDeadlineGroup(new WaitCommand(5),
                    driveTrain.driveBySetpointPercentagesCommand(0.25, 0.3)
                ).until(intake::notePresent), //if it's been 5 seconds or there is a note in the intake
                driveTrain.driveBySetpointPercentagesCommand(-0.43, -0.5),
                new MoveArmToSetpointCommand(arm, Arm.SetpointOptions.SHOOT_CLOSE, () -> true), //aim
                new WaitCommand(1.5),
                new IntakeIndexCommand(intake), //fire
                new InstantCommand(shooterCommand::stop) //turn off shooter
            )
        );
    }

    @Override
    public String getName() {
        return "Move Out Mid Two Ring";
    }
}
