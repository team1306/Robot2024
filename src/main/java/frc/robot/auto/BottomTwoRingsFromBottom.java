package frc.robot.auto;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.path.PathPlannerPath;

import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitUntilCommand;
import frc.robot.commands.intake.IntakeIndexCommand;
import frc.robot.subsystems.Arm;
import frc.robot.subsystems.DriveTrain;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.Shooter;
import frc.robot.subsystems.vision.NoteDetector;

public class BottomTwoRingsFromBottom extends SequentialCommandGroup {
    public BottomTwoRingsFromBottom(NoteDetector detector, Intake intake, Shooter shooter, Arm arm, DriveTrain driveTrain) {
        addCommands(
            AutoBuilder.followPath(PathPlannerPath.fromPathFile("Start-Source to Shoot-Bottom")),
            new InstantCommand(() -> {shooter.setTargetSpeed(.79);driveTrain.setSideVoltages(0, 0);}), //spin up shooter
            arm.getPitchControlCommand(driveTrain),
            new WaitUntilCommand(arm::atSetpoint),
            new IntakeIndexCommand(intake), //fire
            new InstantCommand(() -> shooter.setTargetSpeed(0)),
            AutoCommands.getShootBottomToFar5(intake, shooter, arm, driveTrain) //collect close 1 and shoot
        );
    }
    
    @Override
    public String getName() {
        return "Close Top Ring and Top Far Two Rings from Start Top";
    }
}
