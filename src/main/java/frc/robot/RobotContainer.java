// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj2.command.*;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.subsystems.SwerveSubsystem;

import static frc.robot.Constants.*;

public class RobotContainer {
  private final CommandXboxController controller1 = new CommandXboxController(0); // Creates an XboxController on port 1.
  private final CommandXboxController controller2 = new CommandXboxController(1); // Creates an XboxController on port 1.
  private final SwerveSubsystem drivebase = new SwerveSubsystem();

  public RobotContainer() {
    configureBindings();
    Command driveFieldOrientedDirectAngle = drivebase.driveCommand(
        () -> MathUtil.applyDeadband(controller1.getLeftY(), 0),
        () -> MathUtil.applyDeadband(controller1.getLeftX(), 0),
        () -> controller1.getRightX(),
        () -> controller1.getRightY());

    Command driveFieldOrientedAnglularVelocity = drivebase.driveCommand(
        () -> MathUtil.applyDeadband(controller1.getLeftY(), LEFT_Y_DEADBAND),
        () -> MathUtil.applyDeadband(controller1.getLeftX(), LEFT_X_DEADBAND),
        () -> controller1.getRightX() * 0.5);

    Command driveFieldOrientedDirectAngleSim = drivebase.simDriveCommand(
        () -> MathUtil.applyDeadband(controller1.getLeftY(), LEFT_Y_DEADBAND),
        () -> MathUtil.applyDeadband(controller1.getLeftX(), LEFT_X_DEADBAND),
        () -> controller1.getRawAxis(2));

    drivebase.setDefaultCommand(
        !RobotBase.isSimulation() ? driveFieldOrientedDirectAngle : driveFieldOrientedDirectAngleSim);
    drivebase.setDefaultCommand(driveFieldOrientedDirectAngle);

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
    controller1.a().onTrue((Commands.runOnce(drivebase::zeroGyro)));
  }

  public Command getAutonomousCommand() {
    return new InstantCommand();
  }
}
