package frc.robot.auto;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.path.PathPlannerPath;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.ParallelRaceGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import edu.wpi.first.wpilibj2.command.WaitUntilCommand;
import frc.robot.commands.intake.IntakeIndexCommand;
import frc.robot.commands.shooter.ToggleShooterCommand;
import frc.robot.subsystems.Arm;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.Shooter;

public final class AutoCommands {
    //variables naming guide: get Ring ID + Starting Location

    public static final double waitTime = 1;
    
    public static Command getShooterCommand(Shooter shooter) {
        return new ToggleShooterCommand(() -> .79, Arm.SetpointOptions.SHOOT_CLOSE::getPos, shooter);
    }

    public static SequentialCommandGroup shoot (Intake intake, Shooter shooter) {
        return new SequentialCommandGroup(
            new WaitCommand(1),
            new IntakeIndexCommand(intake) //fire
        
        );
    }

    public static SequentialCommandGroup getSpeakerShootTop (Intake intake) {
        return new SequentialCommandGroup(
            AutoBuilder.followPath(PathPlannerPath.fromPathFile("Shoot-Top to Speaker")),
            new IntakeIndexCommand(intake),
            AutoBuilder.followPath(PathPlannerPath.fromPathFile("Start-2 to Shoot-Top"))
        );
        
    }

    public static SequentialCommandGroup getSpeakerShootBottom (Intake intake) {
        return new SequentialCommandGroup(
            AutoBuilder.followPath(PathPlannerPath.fromPathFile("Shoot-Bottom to Speaker")),
            new IntakeIndexCommand(intake),
            AutoBuilder.followPath(PathPlannerPath.fromPathFile("Start-2 to Shoot-Bottom"))
        );
        
    }

    public static SequentialCommandGroup getFar1ShootTop (Intake intake, Shooter shooter){ 
        final Command shooterCommand = getShooterCommand(shooter);
        return new SequentialCommandGroup(
            AutoBuilder.followPath(PathPlannerPath.fromPathFile("Shoot-Top to Far-1")),
            getIntakeWaiterCommand(intake),
            shooterCommand,
            AutoBuilder.followPath(PathPlannerPath.fromPathFile("Far-1 to Shoot-Top")),
            //new IntakeIndexCommand(intake), //fire
            getSpeakerShootTop(intake), //drive to speaker and fire
            new InstantCommand(shooterCommand::cancel)
            
        );
    }

    public static SequentialCommandGroup getFar2ShootTop (Intake intake, Shooter shooter){
        final Command shooterCommand = getShooterCommand(shooter);
        return new SequentialCommandGroup(
            AutoBuilder.followPath(PathPlannerPath.fromPathFile("Shoot-Top to Far-2")),
            getIntakeWaiterCommand(intake),
            shooterCommand,
            AutoBuilder.followPath(PathPlannerPath.fromPathFile("Far-2 to Shoot-Top")),
            //new IntakeIndexCommand(intake), //fire
            getSpeakerShootTop(intake), //drive to speaker and fire
            new InstantCommand(shooterCommand::cancel)
        );
    }
    
    public static SequentialCommandGroup getFar4ShootBottom (Intake intake, Shooter shooter){ 
        final Command shooterCommand = getShooterCommand(shooter);
        return new SequentialCommandGroup(
            AutoBuilder.followPath(PathPlannerPath.fromPathFile("Shoot-Bottom to Far-4")),
            getIntakeWaiterCommand(intake),
            shooterCommand,
            AutoBuilder.followPath(PathPlannerPath.fromPathFile("Far-4 to Shoot-Bottom")),
            //new IntakeIndexCommand(intake), //fire
            getSpeakerShootBottom(intake), //drive to speaker and fire
            new InstantCommand(shooterCommand::cancel)
        );
    }

    public static SequentialCommandGroup getFar5ShootBottom (Intake intake, Shooter shooter){ 
        final Command shooterCommand = getShooterCommand(shooter);
        return new SequentialCommandGroup(
            AutoBuilder.followPath(PathPlannerPath.fromPathFile("Shoot-Bottom to Far-5")),
            getIntakeWaiterCommand(intake),
            shooterCommand,
            AutoBuilder.followPath(PathPlannerPath.fromPathFile("Far-5 to Shoot-Bottom")),
            //new IntakeIndexCommand(intake), //fire
            getSpeakerShootBottom(intake), //drive to speaker and fire
            new InstantCommand(shooterCommand::cancel)
        );
    }

    public static SequentialCommandGroup getFar3ScanTop (Intake intake, Shooter shooter){
        final Command shooterCommand = getShooterCommand(shooter);
        return new SequentialCommandGroup(
            AutoBuilder.followPath(PathPlannerPath.fromPathFile("Scan-Top to Far-3")),
            getIntakeWaiterCommand(intake),
            shooterCommand,
            AutoBuilder.followPath(PathPlannerPath.fromPathFile("Far-3 to Shoot-Bottom")),
            //new IntakeIndexCommand(intake), //fire
            getSpeakerShootBottom(intake), //drive to speaker and fire
            new InstantCommand(shooterCommand::cancel)
        );
    }

    public static SequentialCommandGroup getFar3ScanBottom (Intake intake, Shooter shooter){
        final Command shooterCommand = getShooterCommand(shooter);
        return new SequentialCommandGroup(
            AutoBuilder.followPath(PathPlannerPath.fromPathFile("Scan-Bottom to Far-3")),
            getIntakeWaiterCommand(intake),
            shooterCommand,
            AutoBuilder.followPath(PathPlannerPath.fromPathFile("Far-3 to Shoot-Top")),
            //new IntakeIndexCommand(intake), //fire
            getSpeakerShootTop(intake), //drive to speaker and fire
            new InstantCommand(shooterCommand::cancel)
        );
    }

    public static SequentialCommandGroup getFar1ScanBottom (Intake intake, Shooter shooter){
        final Command shooterCommand = getShooterCommand(shooter);
        return new SequentialCommandGroup(
            AutoBuilder.followPath(PathPlannerPath.fromPathFile("Scan-Bottom to Far-1")),
            getIntakeWaiterCommand(intake),
            shooterCommand,
            AutoBuilder.followPath(PathPlannerPath.fromPathFile("Far-1 to Shoot-Top")),
            new IntakeIndexCommand(intake), //fire
            new InstantCommand(shooterCommand::cancel)
        );
    }

    public static SequentialCommandGroup getFar2ScanBottom (Intake intake, Shooter shooter){ 
        final Command shooterCommand = getShooterCommand(shooter);
        return new SequentialCommandGroup(
            AutoBuilder.followPath(PathPlannerPath.fromPathFile("Scan-Bottom to Far-2")),
            getIntakeWaiterCommand(intake),
            shooterCommand,
            AutoBuilder.followPath(PathPlannerPath.fromPathFile("Far-2 to Shoot-Top")),
            //new IntakeIndexCommand(intake), //fire
            getSpeakerShootTop(intake), //drive to speaker and fire
            new InstantCommand(shooterCommand::cancel)
        );
    }

    public static SequentialCommandGroup getFar4ScanTop (Intake intake, Shooter shooter){ 
        final Command shooterCommand = getShooterCommand(shooter);
        return new SequentialCommandGroup(
            AutoBuilder.followPath(PathPlannerPath.fromPathFile("Scan-Top to Far-4")),
            getIntakeWaiterCommand(intake),
            shooterCommand,
            AutoBuilder.followPath(PathPlannerPath.fromPathFile("Far-4 to Shoot-Bottom")),
            new IntakeIndexCommand(intake), //fire
            new InstantCommand(shooterCommand::cancel)
        );
    }

    public static SequentialCommandGroup getFar5ScanTop (Intake intake, Shooter shooter){ 
        final Command shooterCommand = getShooterCommand(shooter);
        return new SequentialCommandGroup(
            AutoBuilder.followPath(PathPlannerPath.fromPathFile("Scan-Top to Far-5")),
            getIntakeWaiterCommand(intake),
            shooterCommand,
            AutoBuilder.followPath(PathPlannerPath.fromPathFile("Far-5 to Shoot-Bottom")),
            new IntakeIndexCommand(intake), //fire
            new InstantCommand(shooterCommand::cancel)
        );
    }

    public static SequentialCommandGroup getClose1StartMid (Intake intake, Shooter shooter) {
        final Command shooterCommand = getShooterCommand(shooter);
        return new SequentialCommandGroup(
            shooterCommand,
            AutoBuilder.followPath(PathPlannerPath.fromPathFile("Start-Mid to Close-1")),
            getIntakeWaiterCommand(intake),
            new IntakeIndexCommand(intake), //fire
            new InstantCommand(shooterCommand::cancel)
        );
    }

    public static SequentialCommandGroup getClose2Close1 (Intake intake, Shooter shooter) {
        final Command shooterCommand = getShooterCommand(shooter);
        return new SequentialCommandGroup(
            shooterCommand,
            AutoBuilder.followPath(PathPlannerPath.fromPathFile("Close-1 to Close-2")),
            getIntakeWaiterCommand(intake),
            new IntakeIndexCommand(intake), //fire
            new InstantCommand(shooterCommand::cancel)
        );
    }

    public static SequentialCommandGroup getClose3Close2 (Intake intake, Shooter shooter) {
        final Command shooterCommand = getShooterCommand(shooter);
        return new SequentialCommandGroup(
            shooterCommand,
            AutoBuilder.followPath(PathPlannerPath.fromPathFile("Close-2 to Close-3")),
            getIntakeWaiterCommand(intake),
            new IntakeIndexCommand(intake), //fire
            new InstantCommand(shooterCommand::cancel)
        );
    }


    public static Command getIntakeWaiterCommand(Intake intake) {
        return new ParallelRaceGroup(new WaitUntilCommand(intake::notePresent), new WaitCommand(waitTime));
    }
}
