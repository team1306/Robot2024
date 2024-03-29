// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import com.pathplanner.lib.commands.PathPlannerAuto;
import edu.wpi.first.wpilibj.DigitalOutput;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.FunctionalCommand;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import frc.robot.auto.AutonomousFactory;
import frc.robot.auto.CloseRingsFromStartMid;
import frc.robot.auto.FarRingsFromStartBottom;
import frc.robot.auto.FarRingsFromStartMid;
import frc.robot.auto.FarRingsFromStartTop;
import frc.robot.auto.JustShoot;
import frc.robot.auto.MoveOutLeft;
import frc.robot.auto.MoveOutMid;
import frc.robot.auto.MoveOutMidTwoRing;
import frc.robot.auto.MoveOutRight;
import frc.robot.auto.MoveOutRightTwoRing;
import frc.robot.commands.arm.MoveArmCommand;
import frc.robot.commands.arm.MoveArmToSetpointCommand;
import frc.robot.commands.drive.ShooterDriveCommand;
import frc.robot.commands.drive.TeleopDriveCommand;
import frc.robot.commands.intake.IntakeDriverCommand;
import frc.robot.commands.intake.ToggleIntakeCommand;
import frc.robot.commands.shooter.ToggleShooterCommand;
import frc.robot.subsystems.Arm;
import frc.robot.subsystems.DriveTrain;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.Shooter;
import frc.robot.subsystems.vision.NoteDetector;
import frc.robot.util.DashboardGetter;
import frc.robot.util.Utilities;

import static frc.robot.util.Utilities.removeAndCancelDefaultCommand;
import static frc.robot.util.Utilities.WrappedDouble;

import java.util.function.BooleanSupplier;


public class RobotContainer {
  private final CommandXboxController controller1 = new CommandXboxController(0); // Creates an XboxController on port 1.
  private final CommandXboxController controller2 = new CommandXboxController(1); // Creates an XboxController on port 1.

  private final SendableChooser<AutonomousFactory> autoChooser = new SendableChooser<>();
  private final SendableChooser<Runnable> drivetrainTestModeChooser = new SendableChooser<>();
  
  public final DriveTrain driveTrain;
  public final Intake intake;
  public final Arm arm;

  private final Shooter shooter;
  public final TeleopDriveCommand teleopDriveCommand;
  public final IntakeDriverCommand intakeDriverCommand;
 
  private final ShooterDriveCommand shooterDriveCommand;
  private final MoveArmCommand moveArmCommand;
  private final ToggleShooterCommand toggleShooterCommand, ampShooterCommand;
  private final ToggleIntakeCommand toggleIntakeCommand;

  private Command autonomousCommand = new InstantCommand() {
    @Override
    public String getName() {
      return "NO AUTO";
    }
  };
  private final BooleanSupplier cancelSetpoint = () -> controller2.getRightY() > 0 || controller2.getRightY() < 0 || controller2.b().getAsBoolean(); // b acts as cancel button
  
  private final DigitalOutput ledRedBlueOutput, notePresentOutput;
  private double beginningAutoWait = 0;

  public RobotContainer() {
    SmartDashboard.putNumber("Beginning Auto Wait", beginningAutoWait);
    /*
    UsbCamera usbCamera = new UsbCamera("USB Camera 0", 0);
    usbCamera.setFPS(16);
    usbCamera.setResolution(320, 240);
    usbCamera.setConnectionStrategy(ConnectionStrategy.kKeepOpen);

    UsbCamera usbCamera2 = new UsbCamera("USB Camera 1", 1);
    usbCamera2.setFPS(16);
    usbCamera2.setResolution(320, 240);
    usbCamera2.setConnectionStrategy(ConnectionStrategy.kKeepOpen);

    VideoSink videoSink = CameraServer.addSwitchedCamera("driver");
    */
    driveTrain = new DriveTrain();
    intake = new Intake();
    shooter = new Shooter();
    arm = new Arm();
    shooterDriveCommand = new ShooterDriveCommand(driveTrain);
    moveArmCommand = new MoveArmCommand(arm, controller2::getRightY);
    toggleIntakeCommand = new ToggleIntakeCommand(intake, controller2.a(), controller2.b());
    intakeDriverCommand = new IntakeDriverCommand(intake, shooter, controller2.b(), arm.getCurrentAngle()::getDegrees);
    teleopDriveCommand = new TeleopDriveCommand(driveTrain, controller1::getLeftTriggerAxis, controller1::getRightTriggerAxis, () -> -controller1.getLeftX());
    toggleShooterCommand = new ToggleShooterCommand(() -> Shooter.peakOutput, shooter);
    ampShooterCommand = new ToggleShooterCommand(() -> Shooter.peakOutput /3D, shooter);
    arm.setDefaultCommand(new MoveArmToSetpointCommand(arm, Arm.SetpointOptions.INTAKE));
    configureBindings();
    
    autoChooser.setDefaultOption("move out mid", (NoteDetector unused,  DriveTrain driveTrain, Shooter shooter, Arm arm, Intake intake) -> new MoveOutMid(driveTrain, shooter, arm, intake));
    autoChooser.addOption("move out left", (NoteDetector unused,  DriveTrain driveTrain, Shooter shooter, Arm arm, Intake intake) -> new MoveOutLeft(driveTrain, shooter, arm, intake));
    autoChooser.addOption("move out right", (NoteDetector unused,  DriveTrain driveTrain, Shooter shooter, Arm arm, Intake intake) -> new MoveOutRight(driveTrain, shooter, arm, intake));
    autoChooser.addOption("move out mid 2", (NoteDetector unused,  DriveTrain driveTrain, Shooter shooter, Arm arm, Intake intake) -> new MoveOutMidTwoRing(driveTrain, shooter, arm, intake));
        autoChooser.addOption("move out right 2", (NoteDetector unused,  DriveTrain driveTrain, Shooter shooter, Arm arm, Intake intake) -> new MoveOutRightTwoRing(driveTrain, shooter, arm, intake));

    autoChooser.addOption("Close Rings from Start Mid", (NoteDetector detector,  DriveTrain driveTrain, Shooter shooter, Arm arm, Intake intake) -> new CloseRingsFromStartMid(detector, intake, shooter, arm));
    autoChooser.addOption("Far Rings from Start Bottom (right around stage)", (NoteDetector detector,  DriveTrain driveTrain, Shooter shooter, Arm arm, Intake intake) -> new FarRingsFromStartBottom(detector, intake, shooter, arm));
    autoChooser.addOption("Far Rings from Start Top (left around stage)", (NoteDetector detector,  DriveTrain driveTrain, Shooter shooter, Arm arm, Intake intake) -> new FarRingsFromStartMid(detector, intake, shooter, arm));
    autoChooser.addOption("Far Rings from Start Mid (left around stage)", (NoteDetector detector,  DriveTrain driveTrain, Shooter shooter, Arm arm, Intake intake) -> new FarRingsFromStartTop(detector, intake, shooter, arm));
    autoChooser.addOption("testPath", (NoteDetector a,  DriveTrain b, Shooter c, Arm d, Intake e) -> new PathPlannerAuto("testPath"));
    autoChooser.addOption("Just Shoot", (NoteDetector unused,  DriveTrain alsoUnused, Shooter shooter, Arm arm, Intake intake) -> new JustShoot(shooter, arm, intake));
    SmartDashboard.putData("auto chooser", autoChooser);

    drivetrainTestModeChooser.setDefaultOption("sysid", this::configureSysIDBindings);
    drivetrainTestModeChooser.addOption("controller", this::bindDrivetrainTeleop);
    drivetrainTestModeChooser.addOption("manual voltage input", () -> {
      final WrappedDouble leftVolts = new WrappedDouble(), rightVolts = new WrappedDouble();
      new FunctionalCommand(() -> {
        DashboardGetter.addGetDoubleData("Left Side Drivetrain Test Voltage", leftVolts.val, a -> leftVolts.val = a);
        DashboardGetter.addGetDoubleData("Right Side Drivetrain Test Voltage", rightVolts.val, a -> rightVolts.val = a);
      }, () -> driveTrain.setSideVoltages(leftVolts.val, rightVolts.val), interrupted -> {}, () -> false, driveTrain).schedule();
    });
    drivetrainTestModeChooser.addOption("none", () -> {});

    SmartDashboard.putData("Drivetrain Test Mode Chooser", drivetrainTestModeChooser);

    notePresentOutput = new DigitalOutput(8);
    ledRedBlueOutput = new DigitalOutput(9);
    notePresentOutput.set(false);
    ledRedBlueOutput.set(true);
    
    Commands.run(() -> notePresentOutput.set(intake.notePresent())).schedule();
    Commands.run(() -> ledRedBlueOutput.set(Utilities.isRedAlliance())).ignoringDisable(true).schedule();
  }

  public void autoWaitGetterPeriodic() {
    beginningAutoWait = SmartDashboard.getNumber("Beginning Auto Wait", beginningAutoWait);
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
    controller1.a().whileTrue(shooterDriveCommand);
    controller1.b().whileTrue(driveTrain.getSetSpeedMultiplierCommand(Constants.SLOW_MODE_SPEED));

    controller2.a().onTrue(new InstantCommand(intakeDriverCommand::buttonPress));
    controller2.x().toggleOnTrue(toggleShooterCommand);
    controller2.y().toggleOnTrue(ampShooterCommand);
    controller2.leftBumper().onTrue(arm.getPitchControlCommand(driveTrain));
    controller2.rightBumper().onTrue(new InstantCommand(intakeDriverCommand::clearNote));
    
    controller2.rightTrigger(0.5).onTrue(Commands.either(
        arm.getPitchControlCommand(driveTrain)
            .andThen(Commands.waitUntil(arm::atSetpoint))
            .andThen(Commands.either(
                Commands.waitUntil(() -> intakeDriverCommand.getState() != IntakeDriverCommand.State.INDEXING)
                .andThen(toggleShooterCommand::stop)
                .andThen(new MoveArmToSetpointCommand(arm, Arm.SetpointOptions.INTAKE)),
            Commands.none(), intakeDriverCommand::buttonPress)),
        Commands.none(), intakeDriverCommand::readyToShoot
    ));

    controller2.povUp().onTrue(new MoveArmToSetpointCommand(arm, Arm.SetpointOptions.AMP, cancelSetpoint));
    controller2.povLeft().onTrue(new MoveArmToSetpointCommand(arm, Arm.SetpointOptions.STAGE_SHOT, cancelSetpoint));
    controller2.povRight().onTrue(new MoveArmToSetpointCommand(arm, Arm.SetpointOptions.SHOOT_CLOSE, cancelSetpoint));
    controller2.povDown().onTrue(new MoveArmToSetpointCommand(arm, Arm.SetpointOptions.INTAKE, cancelSetpoint));

    controller2.rightStick().onTrue(moveArmCommand);
    controller2.back().toggleOnTrue(new InstantCommand(intakeDriverCommand::reset).andThen(toggleIntakeCommand));
  }

  public Command getAutonomousCommand() {
    return autonomousCommand;
  }

  public void loadAuto() {
    final Command internalAutoCommand = autoChooser.getSelected().createAutonomousCommand(new NoteDetector.NoteDetectorPlaceHolder(), driveTrain, shooter, arm, intake);
    autonomousCommand = Commands.waitSeconds(beginningAutoWait).andThen(internalAutoCommand);
    autonomousCommand.setName(internalAutoCommand.getName());
  }

  public void configureSysIDBindings() {
    removeAndCancelDefaultCommand(driveTrain);
    controller1
        .povUp()
        .and(controller1.rightBumper())
        .whileTrue(driveTrain.sysIdQuasistatic(SysIdRoutine.Direction.kForward));
    controller1
        .povRight()
        .and(controller1.rightBumper())
        .whileTrue(driveTrain.sysIdQuasistatic(SysIdRoutine.Direction.kReverse));
    controller1
        .povDown()
        .and(controller1.rightBumper())
        .whileTrue(driveTrain.sysIdDynamic(SysIdRoutine.Direction.kForward));
    controller1
        .povLeft()
        .and(controller1.rightBumper())
        .whileTrue(driveTrain.sysIdDynamic(SysIdRoutine.Direction.kReverse));
  }

  public void bindDrivetrainTestMode() {
    drivetrainTestModeChooser.getSelected().run();
  }

  public void bindDrivetrainTeleop() {
    driveTrain.setDefaultCommand(teleopDriveCommand);
  }
}
