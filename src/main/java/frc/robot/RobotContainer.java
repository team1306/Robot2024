// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.commands.IntakeDriverCommand;
import frc.robot.commands.arm.MoveArmCommand;
import frc.robot.commands.drive.ShooterDriveCommand;
import frc.robot.commands.drive.TeleopDriveCommand;
import frc.robot.commands.shooter.ShootCommand;
import frc.robot.commands.shooter.ShooterPitchControlCommand;
import frc.robot.subsystems.Arm;
import frc.robot.subsystems.DriveTrain;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.Shooter;
import frc.robot.Constants;

public class RobotContainer {

  XboxController controller = new XboxController(1); // Creates an XboxController on port 1.
  Trigger aButton = new JoystickButton(controller, XboxController.Button.kA.value); // Creates a new JoystickButton object for the `A` button on controller

  private DriveTrain driveTrain;
  private Intake intake;
  private Shooter shooter; 
  private Arm arm;
  
  private MoveArmCommand moveArmCommand;
  private ShooterDriveCommand shooterDriveCommand;
  private ShootCommand shootCommand;
  private ShooterPitchControlCommand shooterPitchControlCommand;
  private IntakeDriverCommand intakeDriverCommand;
  private TeleopDriveCommand teleopDriveCommand;
  
  
  public RobotContainer() {
    driveTrain = new DriveTrain();
    intake = new Intake();
    shooter = new Shooter();
    arm = new Arm();

    //Dont pass controller values and null, pass suppliers
    moveArmCommand = new MoveArmCommand(driveTrain, arm, null);
    shooterDriveCommand = new ShooterDriveCommand(driveTrain, shootCommand);
    shootCommand = new ShootCommand(shooter, intake);
    shooterPitchControlCommand = new ShooterPitchControlCommand(arm, shootCommand);
    intakeDriverCommand = new IntakeDriverCommand(intake);
    teleopDriveCommand = new TeleopDriveCommand(driveTrain, null);
    // Example Pathplanner named command registration 
    // NamedCommands.registerCommand("ShootCommand", shooterPitchControlCommand);
    
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
    aButton.whileTrue(driveTrain.getSetSpeedMultiplierCommand(SLOW_MODE_SPEED));
    //driveCommand = new TeleopDriveCommand(driveTrain, controller);
    //driveTrain.setDefaultCommand(driveCommand);
  }

  public Command getAutonomousCommand() {
    return Commands.print("No autonomous command configured");
  }
}
