package frc.robot.auto;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.path.PathPlannerPath;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.ParallelDeadlineGroup;
import edu.wpi.first.wpilibj2.command.ParallelRaceGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import edu.wpi.first.wpilibj2.command.WaitUntilCommand;
import frc.robot.commands.drive.ShooterDriveCommand;
import frc.robot.commands.intake.IntakeDriverCommand;
import frc.robot.commands.intake.IntakeIndexCommand;
import frc.robot.commands.shooter.ToggleShooterCommand;
import frc.robot.subsystems.Arm;
import frc.robot.subsystems.DriveTrain;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.Shooter;
import frc.robot.util.Utilities;

public final class AutoCommands {
    public static boolean firstSpinUp = true;
    //variables naming guide: get Ring ID + Starting Location

    // arm.getPitchControlCommand(driveTrain).andThen(Commands.waitUntil(arm::atSetpoint)), AIM

    public static final double waitTime = 1;
    
    public static Command getShooterCommand(Shooter shooter) {
        return new ToggleShooterCommand(() -> .79, shooter);
    }

    public static SequentialCommandGroup shoot (Intake intake, Shooter shooter) {
        return new SequentialCommandGroup(
            new WaitCommand(1),
            new IntakeIndexCommand(intake) //fire
        
        );
    }

    public static SequentialCommandGroup getSpeakerShootTop (Intake intake, Arm arm, DriveTrain driveTrain) {
        return new SequentialCommandGroup(
            AutoBuilder.followPath(PathPlannerPath.fromPathFile("Shoot-Top to Speaker")),
            arm.getPitchControlCommand(driveTrain).andThen(Commands.waitUntil(arm::atSetpoint)), //aim
            new IntakeIndexCommand(intake), //fire
            AutoBuilder.followPath(PathPlannerPath.fromPathFile("Start-2 to Shoot-Top"))
        );
        
    }

    public static SequentialCommandGroup getSpeakerShootBottom (Intake intake, Arm arm, DriveTrain driveTrain) {
        return new SequentialCommandGroup(
            AutoBuilder.followPath(PathPlannerPath.fromPathFile("Shoot-Bottom to Speaker")),
            arm.getPitchControlCommand(driveTrain).andThen(Commands.waitUntil(arm::atSetpoint)), //aim
            new IntakeIndexCommand(intake), //fire
            AutoBuilder.followPath(PathPlannerPath.fromPathFile("Start-2 to Shoot-Bottom"))
        );
        
    }

    public static SequentialCommandGroup getFar1ShootTop (Intake intake, Shooter shooter, Arm arm, DriveTrain driveTrain){ 
        final Command shooterCommand = getShooterCommand(shooter);
        return new SequentialCommandGroup(
            AutoBuilder.followPath(PathPlannerPath.fromPathFile("Shoot-Top to Far-1")),
            getIntakeWaiterCommand(intake),
            shooterCommand,
            AutoBuilder.followPath(PathPlannerPath.fromPathFile("Far-1 to Shoot-Top")),
            getSpeakerShootTop(intake, arm, driveTrain), //drive to speaker and fire
            new InstantCommand(shooterCommand::cancel)
            
        );
    }

    public static SequentialCommandGroup getFar2ShootTop (Intake intake, Shooter shooter, Arm arm, DriveTrain driveTrain){
        final Command shooterCommand = getShooterCommand(shooter);
        return new SequentialCommandGroup(
            AutoBuilder.followPath(PathPlannerPath.fromPathFile("Shoot-Top to Far-2")),
            getIntakeWaiterCommand(intake),
            shooterCommand,
            AutoBuilder.followPath(PathPlannerPath.fromPathFile("Far-2 to Shoot-Top")),
            getSpeakerShootTop(intake, arm, driveTrain), //drive to speaker and fire
            new InstantCommand(shooterCommand::cancel)
        );
    }
    
    public static SequentialCommandGroup getFar4ShootBottom (Intake intake, Shooter shooter, Arm arm, DriveTrain driveTrain){ 
        final Command shooterCommand = getShooterCommand(shooter);
        return new SequentialCommandGroup(
            AutoBuilder.followPath(PathPlannerPath.fromPathFile("Shoot-Bottom to Far-4")),
            getIntakeWaiterCommand(intake),
            shooterCommand,
            AutoBuilder.followPath(PathPlannerPath.fromPathFile("Far-4 to Shoot-Bottom")),
            getSpeakerShootBottom(intake, arm, driveTrain), //drive to speaker and fire
            new InstantCommand(shooterCommand::cancel)
        );
    }

    public static SequentialCommandGroup getFar5ShootBottom (Intake intake, Shooter shooter, Arm arm, DriveTrain driveTrain){ 
        final Command shooterCommand = getShooterCommand(shooter);
        return new SequentialCommandGroup(
            AutoBuilder.followPath(PathPlannerPath.fromPathFile("Shoot-Bottom to Far-5")),
            getIntakeWaiterCommand(intake),
            shooterCommand,
            AutoBuilder.followPath(PathPlannerPath.fromPathFile("Far-5 to Shoot-Bottom")),
            getSpeakerShootBottom(intake, arm, driveTrain), //drive to speaker and fire
            new InstantCommand(shooterCommand::cancel)
        );
    }

    public static SequentialCommandGroup getFar3ScanTop (Intake intake, Shooter shooter, Arm arm, DriveTrain driveTrain){
        final Command shooterCommand = getShooterCommand(shooter);
        return new SequentialCommandGroup(
            AutoBuilder.followPath(PathPlannerPath.fromPathFile("Scan-Top to Far-3")),
            getIntakeWaiterCommand(intake),
            shooterCommand,
            AutoBuilder.followPath(PathPlannerPath.fromPathFile("Far-3 to Shoot-Bottom")),
            arm.getPitchControlCommand(driveTrain).andThen(Commands.waitUntil(arm::atSetpoint)),
            getSpeakerShootBottom(intake, arm, driveTrain), //drive to speaker and fire
            new InstantCommand(shooterCommand::cancel)
        );
    }

    public static SequentialCommandGroup getFar3ScanBottom (Intake intake, Shooter shooter, Arm arm, DriveTrain driveTrain){
        final Command shooterCommand = getShooterCommand(shooter);
        return new SequentialCommandGroup(
            AutoBuilder.followPath(PathPlannerPath.fromPathFile("Scan-Bottom to Far-3")),
            getIntakeWaiterCommand(intake),
            shooterCommand,
            AutoBuilder.followPath(PathPlannerPath.fromPathFile("Far-3 to Shoot-Top")),
            getSpeakerShootTop(intake, arm, driveTrain), //drive to speaker and fire
            new InstantCommand(shooterCommand::cancel)
        );
    }

    public static SequentialCommandGroup getFar1ScanBottom (Intake intake, Shooter shooter, Arm arm, DriveTrain driveTrain){
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

    public static SequentialCommandGroup getFar2ScanBottom (Intake intake, Shooter shooter, Arm arm, DriveTrain driveTrain){ 
        final Command shooterCommand = getShooterCommand(shooter);
        return new SequentialCommandGroup(
            AutoBuilder.followPath(PathPlannerPath.fromPathFile("Scan-Bottom to Far-2")),
            getIntakeWaiterCommand(intake),
            shooterCommand,
            AutoBuilder.followPath(PathPlannerPath.fromPathFile("Far-2 to Shoot-Top")),
            getSpeakerShootTop(intake, arm, driveTrain), //drive to speaker and fire
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

    @SafeVarargs
    public static Command followPathsWhileIntakingAndThenShoot(Intake intake, Shooter shooter, Arm arm, DriveTrain driveTrain, boolean shooterAfterFirstPath, String... pathNames) {
        System.out.println(pathNames[0]);
        final IntakeDriverCommand intakeDriverCommand = new IntakeDriverCommand(intake, shooter, () -> arm.getCurrentAngle().getDegrees(), IntakeDriverCommand.State.POWERED_NO_ELEMENT);
        final SequentialCommandGroup pathsAndShooter = new SequentialCommandGroup();
        final ToggleShooterCommand shooterCommand = new ToggleShooterCommand(() -> 1, shooter);
        final InstantCommand scheduleShooterCommand = new InstantCommand(shooterCommand::schedule);
        
        for (int i = 0; i < pathNames.length; ++i) {
            pathsAndShooter.addCommands(AutoBuilder.followPath(PathPlannerPath.fromPathFile(pathNames[i])));
            if (i == 0 && shooterAfterFirstPath) {
                // pathsAndShooter.addCommands(scheduleShooterCommand);
            }
        }
        // above for loop should be replaced with the below
        // pathsAndShooter.addCommands((Command[]) Utilities.map(path -> AutoBuilder.followPath(PathPlannerPath.fromPathFile(path)), pathNames).toArray());

        if (!shooterAfterFirstPath || pathNames.length == 0) {
            // pathsAndShooter.addCommands(scheduleShooterCommand);
        }

        ParallelDeadlineGroup command = new ParallelDeadlineGroup(
            new SequentialCommandGroup(
                new InstantCommand(() -> arm.setTargetAngle(Rotation2d.fromDegrees(2))),
                new ParallelRaceGroup(new WaitUntilCommand(intake::notePresent), pathsAndShooter),
                new WaitCommand(0.2),
                scheduleShooterCommand,
                new InstantCommand(()->driveTrain.setSideVoltages(0, 0)),
                new ShooterDriveCommand(driveTrain),
                new InstantCommand(()->driveTrain.setSideVoltages(0, 0)),
                arm.getPitchControlCommand(driveTrain),
                new WaitUntilCommand(() -> Math.abs(shooter.getTopRPM()) > 4250 && arm.atSetpoint()),
                new InstantCommand(intakeDriverCommand::buttonPress),
                new WaitUntilCommand(intakeDriverCommand::noLongerIndexing),
                new InstantCommand(() -> {
                    shooterCommand.stop();
                    driveTrain.setSideVoltages(0, 0);
                })
            ),
            intakeDriverCommand
        );
        return command;
    }

    @SafeVarargs
    public static Command followPathsWhileIntakingAndThenShoot(Intake intake, Shooter shooter, Arm arm, DriveTrain driveTrain, String... pathNames) {
        return followPathsWhileIntakingAndThenShoot(intake, shooter, arm, driveTrain, true, pathNames);
    }

    public static Command getStartMidToClose1 (Intake intake, Shooter shooter, Arm arm, DriveTrain driveTrain) {
        return followPathsWhileIntakingAndThenShoot(intake, shooter, arm, driveTrain, "Start-Mid to Close-1");
    }

    public static Command getStartTopToClose1 (Intake intake, Shooter shooter, Arm arm, DriveTrain driveTrain) {
        return followPathsWhileIntakingAndThenShoot(intake, shooter, arm, driveTrain, "Start-1 to Close-1");
    }

    public static Command getClose1ToClose2 (Intake intake, Shooter shooter, Arm arm, DriveTrain driveTrain) {
        return followPathsWhileIntakingAndThenShoot(intake, shooter, arm, driveTrain, "Close-1 to Close-2");
    }

    public static Command getClose2ToClose3 (Intake intake, Shooter shooter, Arm arm, DriveTrain driveTrain) {
        return followPathsWhileIntakingAndThenShoot(intake, shooter, arm, driveTrain, false, "Close-2 to Close-3");
    }

    public static Command getStartBottomToClose3 (Intake intake, Shooter shooter, Arm arm, DriveTrain driveTrain) {
        return followPathsWhileIntakingAndThenShoot(intake, shooter, arm, driveTrain, "Start-Bottom to Close-3");
    }

    public static Command getClose3ToClose2 (Intake intake, Shooter shooter, Arm arm, DriveTrain driveTrain) {
        return followPathsWhileIntakingAndThenShoot(intake, shooter, arm, driveTrain, false, "Close-3 to Close-2 P1", "Close-3 to Close-2 P2");
    }

    public static Command getClose2ToClose1 (Intake intake, Shooter shooter, Arm arm, DriveTrain driveTrain) {
        return followPathsWhileIntakingAndThenShoot(intake, shooter, arm, driveTrain, "Close-2 to Close-1");
    }

    public static Command getClose1ToFar1ToShootMidTop(Intake intake, Shooter shooter, Arm arm, DriveTrain driveTrain) {
        return followPathsWhileIntakingAndThenShoot(intake, shooter, arm, driveTrain, "Close-1 to Far-1", "Far-1 to Shoot-MidTop");
    }

    public static Command getShootMidTopToFar2ToShootMidTop(Intake intake, Shooter shooter, Arm arm, DriveTrain driveTrain) {
        return followPathsWhileIntakingAndThenShoot(intake, shooter, arm, driveTrain, "Shoot-MidTop to Far-2", "Far-2 to Shoot-MidTop");
    }

    public static Command getStartSourceToShootBottom(Intake intake, Shooter shooter, Arm arm, DriveTrain driveTrain) {
        return followPathsWhileIntakingAndThenShoot(intake, shooter, arm, driveTrain, "Shoot-MidTop to Far-2", "Far-2 to Shoot-MidTop");
    }

    public static Command getIntakeWaiterCommand(Intake intake) {
        return new ParallelRaceGroup(new WaitUntilCommand(intake::notePresent), new WaitCommand(waitTime));
    }
    
}
