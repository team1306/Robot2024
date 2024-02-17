// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.commands.IntakeDriverCommand;
import frc.robot.commands.arm.MoveArmCommand;
import frc.robot.commands.drive.ShooterDriveCommand;
import frc.robot.commands.drive.TeleopDriveCommand;
import frc.robot.commands.shooter.NoteIndexingCommand;
import frc.robot.commands.shooter.ShooterPitchControlCommand;
import frc.robot.subsystems.Arm;
import frc.robot.subsystems.DriveTrain;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.Shooter;
import static frc.robot.Constants.*;

public class RobotContainer {

  private CommandXboxController controller1 = new CommandXboxController(0); // Creates an XboxController on port 1.
  private CommandXboxController controller2 = new CommandXboxController(1); // Creates an XboxController on port 1.

  private DriveTrain driveTrain;
  private Intake intake;
  private Shooter shooter; 
  Arm arm;
  
  private MoveArmCommand moveArmCommand;
  private ShooterDriveCommand shooterDriveCommand;
  private NoteIndexingCommand indexNoteCommand;
  private ShooterPitchControlCommand shooterPitchControlCommand;
  private TeleopDriveCommand teleopDriveCommand;
  
  
  public RobotContainer() {
    // driveTrain = new DriveTrain();
    // intake = new Intake();
    shooter = new Shooter();
    arm = new Arm();

    moveArmCommand = new MoveArmCommand(arm, () -> Math.max(controller2.getRightY(), 0.03));
    //indexNoteCommand = new NoteIndexingCommand(intake);
    // shooterDriveCommand = new ShooterDriveCommand(driveTrain, shooter, indexNoteCommand);
    // shooterPitchControlCommand = new ShooterPitchControlCommand(arm, shooterDriveCommand);
    // teleopDriveCommand = new TeleopDriveCommand(driveTrain, () -> controller1.getRightTriggerAxis(), () -> controller1.getLeftTriggerAxis(), () -> -controller1.getLeftX());
    // Example Pathplanner named command registration 
    // NamedCommands.registerCommand("ShootCommand", shooterPitchControlCommand);
    
    arm.setDefaultCommand(moveArmCommand);
    // driveTrain.setDefaultCommand(teleopDriveCommand);
    configureBindings();
  }

  /**
     * Use this method to define your trigger->command mappings. Triggers can be created via the
     * {@link Trigger#Trigger(java.util.function.BooleanSupplier)} constructor with an arbitrary
     * predicate, or via the named factories in {@link
     * edu.wpi.first.wpilibj2.command.button.CommandGenericHID}'s subclasses for {@link
     * CommandXboxController Xbox}/{@link edu.wpi.first.wpilibj2.command.button.CommandPS4Controller
     * PS4} controllers or {@link edu.wpi.first.wpilibj2.command.button.CommandJoystick Flight
     * joysticks}.
     */
  private void configureBindings() {
    //controller1.a().whileTrue(driveTrain.getSetSpeedMultiplierCommand(SLOW_MODE_SPEED));
    //controller2.b().toggleOnTrue(new IntakeDriverCommand(intake));
    //controller2.y().onTrue(indexNoteCommand);
    controller2.x().toggleOnTrue(shooter.getToggleShooterCommand(() -> Shooter.PEAK_OUTPUT));
    // controller2.x().onTrue(shooterPitchControlCommand);
  }

  public Command getAutonomousCommand() {
    return Commands.print("No autonomous command configured");
  }
}
