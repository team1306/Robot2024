// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.commands.arm.MoveArmCommand;
import frc.robot.commands.arm.MoveArmToSetpointCommand;
import frc.robot.commands.climber.ClimberDriverCommand;
import frc.robot.commands.drive.TeleopDriveCommand;
import frc.robot.commands.intake.IntakeDriverCommand;
import frc.robot.commands.shooter.NoteIndexingCommand;
import frc.robot.commands.shooter.ShooterDriveCommand;
import frc.robot.commands.shooter.ShooterPitchControlCommand;
import frc.robot.subsystems.Arm;
import frc.robot.subsystems.Climber;
import frc.robot.subsystems.DriveTrain;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.Shooter;

import java.util.function.BooleanSupplier;

import com.pathplanner.lib.commands.PathPlannerAuto;

public class RobotContainer {

  private CommandXboxController controller1 = new CommandXboxController(0); // Creates an XboxController on port 1.
  private CommandXboxController controller2 = new CommandXboxController(1); // Creates an XboxController on port 1.

  private DriveTrain driveTrain;
  private Intake intake;
  private Shooter shooter; 
  private Climber climber;
  public Arm arm;
  
  public MoveArmCommand moveArmCommand;
  private ShooterDriveCommand shooterDriveCommand;
  private NoteIndexingCommand indexNoteCommand;
  private ShooterPitchControlCommand shooterPitchControlCommand;
  private TeleopDriveCommand teleopDriveCommand;
  private IntakeDriverCommand intakeDriverCommand;
  private ClimberDriverCommand climberDriverCommand;
  
  private final BooleanSupplier cancelSetpoint = () -> controller2.getRightY() > 0 || controller2.getRightY() < 0 || controller2.b().getAsBoolean(); // b acts as cancel button
  
  public RobotContainer() {
    driveTrain = new DriveTrain();
    intake = new Intake();
    shooter = new Shooter();
    arm = new Arm();
    moveArmCommand = new MoveArmCommand(arm, () -> controller2.getRightY());
    //indexNoteCommand = new NoteIndexingCommand(intake);
    shooterDriveCommand = new ShooterDriveCommand(driveTrain, shooter, indexNoteCommand);
    shooterPitchControlCommand = new ShooterPitchControlCommand(arm, shooterDriveCommand);
    intakeDriverCommand = new IntakeDriverCommand(intake);
    climberDriverCommand = new ClimberDriverCommand(climber);
    teleopDriveCommand = new TeleopDriveCommand(driveTrain, () -> controller1.getLeftTriggerAxis(), () -> controller1.getRightTriggerAxis(), () -> -controller1.getLeftX());
    // Example Pathplanner named command registration 
    // NamedCommands.registerCommand("ShootCommand", shooterPitchControlCommand);

    intake.setDefaultCommand(intakeDriverCommand);
    climber.setDefaultCommand(climberDriverCommand);
    arm.setDefaultCommand(moveArmCommand);
    driveTrain.setDefaultCommand(teleopDriveCommand);
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

    controller2.y().onTrue(new InstantCommand(intakeDriverCommand::buttonPress));
    controller2.x().toggleOnTrue(shooter.getToggleShooterCommand(() -> Shooter.PEAK_OUTPUT));

    controller2.povUp().onTrue(new MoveArmToSetpointCommand(arm, Arm.Setpoint.AMP, cancelSetpoint));
    controller2.povLeft().onTrue(new MoveArmToSetpointCommand(arm, Arm.Setpoint.STAGE_SHOT, cancelSetpoint));
    controller2.povRight().onTrue(new MoveArmToSetpointCommand(arm, Arm.Setpoint.SHOOT_CLOSE, cancelSetpoint));
    controller2.povDown().onTrue(new MoveArmToSetpointCommand(arm, Arm.Setpoint.INTAKE, cancelSetpoint));
    controller2.a().onTrue(shooterPitchControlCommand);

    controller2.b().onTrue(new InstantCommand(climberDriverCommand::buttonPress));
  }
}
