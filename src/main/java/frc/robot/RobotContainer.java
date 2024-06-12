// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import com.pathplanner.lib.commands.PathPlannerAuto;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.DifferentialDriveWheelSpeeds;
import edu.wpi.first.util.sendable.Sendable;
import edu.wpi.first.wpilibj.DigitalOutput;
import edu.wpi.first.wpilibj.event.EventLoop;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.*;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.POVButton;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import frc.robot.auto.AutoCommands;
import frc.robot.auto.AutonomousFactory;
import frc.robot.auto.BottomTwoRingsFromBottom;
import frc.robot.auto.CloseRingsFromStartBottom;
import frc.robot.auto.CloseRingsFromStartMid;
import frc.robot.auto.CloseRingsFromStartTop;
import frc.robot.auto.CloseTopRingAndTopFarTwoRingsFromStartTop;
import frc.robot.auto.FarRingsFromStartBottom;
import frc.robot.auto.FarRingsFromStartMid;
import frc.robot.auto.FarRingsFromStartTop;
import frc.robot.auto.FunnyMid;
import frc.robot.auto.JustShoot;
import frc.robot.auto.MoveOutLeft;
import frc.robot.auto.MoveOutMid;
import frc.robot.auto.MoveOutMidTwoRing;
import frc.robot.auto.MoveOutRight;
import frc.robot.auto.MoveOutRightTwoRing;
import frc.robot.commands.VibrateControllersCommand;
import frc.robot.commands.VibrateControllersCommand.HIDSubsystem;
import frc.robot.commands.arm.MoveArmCommand;
import frc.robot.commands.arm.MoveArmToSetpointCommand;
import frc.robot.commands.drive.ShooterDriveCommand;
import frc.robot.commands.drive.AimBotCommand;
import frc.robot.commands.drive.AimToAprilTagCommand;
import frc.robot.commands.drive.ChrisDriveCommand;
import frc.robot.commands.drive.JoystickDriveCommand;
import frc.robot.commands.intake.IntakeDriverCommand;
import frc.robot.commands.intake.ToggleIntakeCommand;
import frc.robot.commands.outreach.FireCommand;
import frc.robot.commands.outreach.IntakeCommand;
import frc.robot.commands.outreach.ShooterSpeedCommand;
import frc.robot.commands.shooter.ToggleShooterCommand;
import frc.robot.subsystems.Arm;
import frc.robot.subsystems.DriveTrain;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.Shooter;
import frc.robot.subsystems.vision.NoteDetector;
import frc.robot.util.DashboardGetter;
import frc.robot.util.Utilities;
import frc.robot.util.Utilities.WrappedDouble;
import frc.robot.util.Utilities.WrappedInteger;

import static frc.robot.util.Utilities.removeAndCancelDefaultCommand;
import static frc.robot.Constants.LOOP_TIME_SECONDS;
import static frc.robot.util.Utilities.WrappedDouble;

import java.util.ArrayList;
import java.util.List;
import java.util.function.BooleanSupplier;


public class RobotContainer {
  private final CommandXboxController controller1 = new CommandXboxController(0); // Creates an XboxController on port 1.
  private final CommandXboxController controller2 = new CommandXboxController(1); // Creates an XboxController on port 1.

  private final SendableChooser<AutonomousFactory> autoChooser = new SendableChooser<>();
  private final SendableChooser<Runnable> drivetrainTestModeChooser = new SendableChooser<>();
  private final SendableChooser<Runnable> buttonBindingsChooser = new SendableChooser<>();

  private final EventLoop safeP1EventLoop = new EventLoop();
  private final EventLoop safeP2EventLoop = new EventLoop();
  private final EventLoop skilledEventLoop = new EventLoop();
  private final EventLoop superviseEventLoop = new EventLoop();

  
  public final DriveTrain driveTrain;
  public final Intake intake;
  public final Arm arm;

  private Command currentAdjustmentCommand = null;
  private final Shooter shooter;
  public final JoystickDriveCommand teleopDriveCommand;
  public final IntakeDriverCommand intakeDriverCommand;
 
  private final ShooterDriveCommand shooterDriveCommand;
  private final MoveArmCommand moveArmCommand;
  private final ToggleShooterCommand toggleShooterCommand, ampShooterCommand;
  private final IntakeCommand intakeCommand;
  private final FireCommand fireCommand;
  private final ToggleIntakeCommand toggleIntakeCommand;
  private final AimBotCommand aimToAprilTagCommand;
  private final EventLoop hapticLoop = new EventLoop();

  private final List<Command> commandQueueOnStart = new ArrayList<>();
  
  private Command autonomousCommand = new InstantCommand() {
    @Override
    public String getName() {
      return "NO AUTO";
    }
  };
  private final BooleanSupplier cancelSetpoint = () -> controller2.getRightY() > 0 || controller2.getRightY() < 0 || controller2.b().getAsBoolean(); // b acts as cancel button
  
  private final DigitalOutput ledRedBlueOutput, notePresentOutput;
  private double beginningAutoWait = 0;

  private Runnable lastFunMode;

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
    intakeDriverCommand = new IntakeDriverCommand(intake, shooter, controller2.b(), () -> arm.getCurrentAngle().getDegrees());
    teleopDriveCommand = new JoystickDriveCommand(driveTrain, () -> -controller1.getLeftY(), () -> -controller1.getRightX());
    toggleShooterCommand = new ToggleShooterCommand(() -> Shooter.peakOutput, shooter);
    ampShooterCommand = new ToggleShooterCommand(() -> Shooter.peakOutput /3D, shooter);
    aimToAprilTagCommand = new AimBotCommand(driveTrain, shooter, intake, arm, () -> -controller1.getLeftY(), () -> -controller1.getRightX());

    fireCommand = new FireCommand(intake, shooter);
    intakeCommand = new IntakeCommand(intake);
    intake.setDefaultCommand(intakeCommand);
    arm.setDefaultCommand(new MoveArmToSetpointCommand(arm, Arm.SetpointOptions.INTAKE));
    configureBindings();
    
    autoChooser.addOption("move out mid", (NoteDetector unused,  DriveTrain driveTrain, Shooter shooter, Arm arm, Intake intake) -> new MoveOutMid(driveTrain, shooter, arm, intake));
    autoChooser.addOption("move out left", (NoteDetector unused,  DriveTrain driveTrain, Shooter shooter, Arm arm, Intake intake) -> new MoveOutLeft(driveTrain, shooter, arm, intake));
    autoChooser.addOption("move out right", (NoteDetector unused,  DriveTrain driveTrain, Shooter shooter, Arm arm, Intake intake) -> new MoveOutRight(driveTrain, shooter, arm, intake));
    autoChooser.addOption("move out mid 2", (NoteDetector unused,  DriveTrain driveTrain, Shooter shooter, Arm arm, Intake intake) -> new MoveOutMidTwoRing(driveTrain, shooter, arm, intake));
    autoChooser.addOption("move out right 2", (NoteDetector unused,  DriveTrain driveTrain, Shooter shooter, Arm arm, Intake intake) -> new MoveOutRightTwoRing(driveTrain, shooter, arm, intake));

    autoChooser.setDefaultOption("Close Rings from Start Mid", (NoteDetector detector,  DriveTrain driveTrain, Shooter shooter, Arm arm, Intake intake) -> new CloseRingsFromStartMid(detector, intake, shooter, arm, driveTrain));
    autoChooser.addOption("Close Rings from Start Bottom", (NoteDetector detector,  DriveTrain driveTrain, Shooter shooter, Arm arm, Intake intake) -> new CloseRingsFromStartBottom(detector, intake, shooter, arm, driveTrain));
    autoChooser.addOption("Close Rings from Start Top", (NoteDetector detector,  DriveTrain driveTrain, Shooter shooter, Arm arm, Intake intake) -> new CloseRingsFromStartTop(detector, intake, shooter, arm, driveTrain));
    autoChooser.addOption("Close Top Ring and Top Far Two Rings From Start Top", (NoteDetector detector,  DriveTrain driveTrain, Shooter shooter, Arm arm, Intake intake) -> new CloseTopRingAndTopFarTwoRingsFromStartTop(detector, intake, shooter, arm, driveTrain));
    autoChooser.addOption("funny mid", (NoteDetector detector,  DriveTrain driveTrain, Shooter shooter, Arm arm, Intake intake) -> new FunnyMid(detector, intake, shooter, arm, driveTrain));

    autoChooser.addOption("Far Rings from Start Bottom (right around stage)", (NoteDetector detector,  DriveTrain driveTrain, Shooter shooter, Arm arm, Intake intake) -> new FarRingsFromStartBottom(detector, intake, shooter, arm, driveTrain));
    autoChooser.addOption("bottom funny", (NoteDetector detector,  DriveTrain driveTrain, Shooter shooter, Arm arm, Intake intake) -> new BottomTwoRingsFromBottom(detector, intake, shooter, arm, driveTrain));
    autoChooser.addOption("Far Rings from Start Top (left around stage)", (NoteDetector detector,  DriveTrain driveTrain, Shooter shooter, Arm arm, Intake intake) -> new FarRingsFromStartMid(detector, intake, shooter, arm, driveTrain));
    autoChooser.addOption("Far Rings from Start Mid (left around stage)", (NoteDetector detector,  DriveTrain driveTrain, Shooter shooter, Arm arm, Intake intake) -> new FarRingsFromStartTop(detector, intake, shooter, arm, driveTrain));
    autoChooser.addOption("testPath", (NoteDetector a,  DriveTrain b, Shooter c, Arm d, Intake e) -> new PathPlannerAuto("abcdef"));
    autoChooser.addOption("linetest", (NoteDetector a,  DriveTrain b, Shooter c, Arm d, Intake e) -> new PathPlannerAuto("linetest"));
    autoChooser.addOption("Just Shoot", (NoteDetector unused,  DriveTrain alsoUnused, Shooter shooter, Arm arm, Intake intake) -> new JustShoot(shooter, arm, intake));
    SmartDashboard.putData("auto chooser", autoChooser);
    drivetrainTestModeChooser.setDefaultOption("sysid", this::configureSysIDBindings);
    drivetrainTestModeChooser.addOption("controller", this::bindDrivetrainTeleop);
    drivetrainTestModeChooser.addOption("none", () -> {});
    drivetrainTestModeChooser.addOption("manual voltage input", () -> {
      final WrappedDouble leftVolts = new WrappedDouble(), rightVolts = new WrappedDouble();
      new FunctionalCommand(() -> {
        DashboardGetter.addGetDoubleData("Left Side Drivetrain Test Voltage", leftVolts.val, a -> leftVolts.val = a);
        DashboardGetter.addGetDoubleData("Right Side Drivetrain Test Voltage", rightVolts.val, a -> rightVolts.val = a);
      }, () -> driveTrain.setSideVoltages(leftVolts.val, rightVolts.val), interrupted -> {}, () -> false, driveTrain).schedule();
    });
    drivetrainTestModeChooser.addOption("traction current testing", () -> {
      final WrappedDouble currentLimit = new WrappedDouble();
      final WrappedInteger i = new WrappedInteger(-1);
      currentAdjustmentCommand = new FunctionalCommand(
        () -> {
          DashboardGetter.addGetDoubleData("Drivetrain Current Limit", currentLimit.val, a -> currentLimit.val = a);
          bindDrivetrainTeleop();
        }, 
        () -> {
          if (++i.val % (0.5 / LOOP_TIME_SECONDS) == 0) {
            driveTrain.pushCurrentLimitToAllDrivetrainMotors((int) Math.round(currentLimit.val));
          }
        },
        interrupted -> {}, () -> false
      );
      currentAdjustmentCommand.schedule();
    });
    drivetrainTestModeChooser.addOption("manual meters per second input", () -> {
      final WrappedDouble leftMetersS = new WrappedDouble(), rightMetersS = new WrappedDouble();
      new FunctionalCommand(() -> {
        DashboardGetter.addGetDoubleData("Left Side Drivetrain Test Meters per Second", leftMetersS.val, a -> leftMetersS.val = a);
        DashboardGetter.addGetDoubleData("Right Side Drivetrain Test Meters per Second", rightMetersS.val, a -> rightMetersS.val = a);
      }, () -> driveTrain.driveMetersPerSecond(new DifferentialDriveWheelSpeeds(leftMetersS.val, rightMetersS.val)), interrupted -> {}, () -> false, driveTrain).schedule();
    });
    drivetrainTestModeChooser.addOption("none", () -> {});

    SmartDashboard.putData("Drivetrain Test Mode Chooser", drivetrainTestModeChooser);

    buttonBindingsChooser.addOption("Safety w/ 1 Player", this::changeSafeP1Bindings);
    buttonBindingsChooser.addOption("Safety w/ 2 Players", this::changeSafeP2Bindings);
    buttonBindingsChooser.addOption("TRAINING REQUIRED: 1 Skilled Player", this::changeSkilledBindings);
    buttonBindingsChooser.addOption("Supervisor Player", this::changeSuperviseBindings);

    SmartDashboard.putData("Control mode chooser", buttonBindingsChooser);

    notePresentOutput = new DigitalOutput(11);
    ledRedBlueOutput = new DigitalOutput(9);
    notePresentOutput.set(false);
    ledRedBlueOutput.set(true);
    
    Commands.run(() -> notePresentOutput.set(intake.notePresent())).schedule();
    Commands.run(() -> ledRedBlueOutput.set(Utilities.isRedAlliance())).ignoringDisable(true).schedule();
  }

  public void updateButtonBindings(){
    if(lastFunMode != buttonBindingsChooser.getSelected()) buttonBindingsChooser.getSelected().run();
    lastFunMode = buttonBindingsChooser.getSelected();
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
    // controller1.a().whileTrue(shooterDriveCommand);
    // controller1.b().whileTrue(driveTrain.getSetSpeedMultiplierCommand(Constants.SLOW_MODE_SPEED));

    // controller1.rightBumper().onTrue(getAutoShootCommand());

    // controller2.a().onTrue(new InstantCommand(intakeDriverCommand::buttonPress));
    // controller2.x().toggleOnTrue(toggleShooterCommand);
    // controller2.y().toggleOnTrue(ampShooterCommand);
    // controller2.leftBumper().onTrue(arm.getPitchControlCommand(driveTrain));
    // controller2.rightBumper().onTrue(new InstantCommand(intakeDriverCommand::clearNote));

    // //32.5
    // controller2.rightTrigger(0.5).onTrue(new MoveArmToSetpointCommand(arm, Arm.SetpointOptions.OVER_STAGE));
    // controller2.povUp().onTrue(new MoveArmToSetpointCommand(arm, Arm.SetpointOptions.AMP, cancelSetpoint));
    // controller2.povLeft().onTrue(new MoveArmToSetpointCommand(arm, Arm.SetpointOptions.STAGE_SHOT, cancelSetpoint));
    // controller2.povRight().onTrue(new MoveArmToSetpointCommand(arm, Arm.SetpointOptions.SHOOT_CLOSE, cancelSetpoint));
    // controller2.povDown().onTrue(new MoveArmToSetpointCommand(arm, Arm.SetpointOptions.INTAKE, cancelSetpoint));

    // controller2.rightStick().onTrue(moveArmCommand);

    // new Trigger(hapticLoop, intake::notePresent).onTrue(
    //   new ParallelDeadlineGroup(
    //     new WaitCommand(VibrateControllersCommand.RUMBLE_TIME),
    //     new VibrateControllersCommand(
    //       new HIDSubsystem(controller1.getHID()),
    //       new HIDSubsystem(controller2.getHID())
    //     )
    //   )
    // );
    configureSafeP1Bindings();
    configureSafeP2Bindings();
    configureSkilledBindings();
    configureSuperviseBindings();
  }

  private void changeSafeP1Bindings() {
    driveTrain.currentSpeedMultiplier = 0.25;
    System.out.println("Change to Safe P1");
    bindDrivetrainTeleop();
    CommandScheduler.getInstance().setActiveButtonLoop(safeP1EventLoop);
  }
  private void changeSafeP2Bindings(){
    driveTrain.currentSpeedMultiplier = 0.25;
    System.out.println("Change to Safe P2");
    bindDrivetrainTeleop();
    CommandScheduler.getInstance().setActiveButtonLoop(safeP2EventLoop);
  }
  private void changeSkilledBindings(){
    driveTrain.currentSpeedMultiplier = 1;
    System.out.println("Change to Skilled");
    bindDrivetrainTeleop();
    CommandScheduler.getInstance().setActiveButtonLoop(skilledEventLoop);
  }
  private void changeSuperviseBindings(){
    driveTrain.currentSpeedMultiplier = 0.5;
    System.out.println("Change to Supervise");
    CommandScheduler.getInstance().cancelAll();
    unbindDrivetrainTeleop();
    Utilities.removeAndCancelDefaultCommand(intake);
    CommandScheduler.getInstance().setActiveButtonLoop(superviseEventLoop); 
    commandQueueOnStart.add(aimToAprilTagCommand);
  }

  public void teleopScheduleCommand(){
    if(!commandQueueOnStart.isEmpty())
      commandQueueOnStart.forEach(command -> command.schedule());
  }


  private void configureSafeP1Bindings(){
    final ShooterSpeedCommand shooterSpeedCommand = new ShooterSpeedCommand(shooter, () -> controller1.getLeftTriggerAxis()); 
    controller1.a(safeP1EventLoop).onTrue(new InstantCommand(intakeCommand::buttonPress));
    controller1.b(safeP1EventLoop).onTrue(new InstantCommand(intakeCommand::reverseOverride));
    controller1.leftTrigger(0.1, safeP1EventLoop).toggleOnTrue(shooterSpeedCommand);
    controller1.rightTrigger(0.5, safeP1EventLoop).onTrue(fireCommand);
    controller1.pov(0, 0, safeP1EventLoop).onTrue(new MoveArmToSetpointCommand(arm, new Arm.Setpoint.Custom(Rotation2d.fromDegrees(55))));
    controller1.pov(0, 180, safeP1EventLoop).onTrue(new MoveArmToSetpointCommand(arm, new Arm.Setpoint.Custom(Rotation2d.fromDegrees(0))));
  }
  private void configureSafeP2Bindings(){
    final ShooterSpeedCommand shooterSpeedCommand = new ShooterSpeedCommand(shooter, () -> controller2.getLeftTriggerAxis()); 
    controller2.a(safeP2EventLoop).onTrue(new InstantCommand(intakeCommand::buttonPress));
    controller2.b(safeP2EventLoop).onTrue(new InstantCommand(intakeCommand::reverseOverride));
    controller2.leftTrigger(0.1, safeP2EventLoop).toggleOnTrue(shooterSpeedCommand);
    controller2.rightTrigger(0.5, safeP2EventLoop).onTrue(fireCommand);
    controller2.pov(0, 0, safeP2EventLoop).onTrue(new MoveArmToSetpointCommand(arm, new Arm.Setpoint.Custom(Rotation2d.fromDegrees(60))));
    controller2.pov(0, 180, safeP2EventLoop).onTrue(new MoveArmToSetpointCommand(arm, new Arm.Setpoint.Custom(Rotation2d.fromDegrees(0))));
  }
  private void configureSkilledBindings(){
    final ShooterSpeedCommand halfShooterSpeedCommand = new ShooterSpeedCommand(shooter, () -> Shooter.peakOutput / 2); 
    final MoveArmCommand moveArmCommand = new MoveArmCommand(arm, controller1::getLeftY);
    controller1.a(skilledEventLoop).onTrue(new InstantCommand(intakeCommand::buttonPress));
    controller1.b(skilledEventLoop).onTrue(new InstantCommand(intakeCommand::reverseOverride));
    controller1.x(skilledEventLoop).toggleOnTrue(toggleShooterCommand);
    controller1.y(skilledEventLoop).toggleOnTrue(halfShooterSpeedCommand);

    controller1.leftBumper(skilledEventLoop).onTrue(arm.getPitchControlCommand(driveTrain));

    controller1.rightTrigger(0.5, skilledEventLoop).onTrue(fireCommand);
    controller1.pov(0, 0, skilledEventLoop).onTrue(new MoveArmToSetpointCommand(arm, new Arm.Setpoint.Custom(Rotation2d.fromDegrees(60))));
    controller1.pov(0, 90, skilledEventLoop).onTrue(new MoveArmToSetpointCommand(arm, new Arm.Setpoint.Custom(Rotation2d.fromDegrees(30))));
    controller1.pov(0, 180, skilledEventLoop).onTrue(new MoveArmToSetpointCommand(arm, new Arm.Setpoint.Custom(Rotation2d.fromDegrees(0))));
    controller1.pov(0, 270, skilledEventLoop).onTrue(new MoveArmToSetpointCommand(arm, new Arm.Setpoint.Custom(Rotation2d.fromDegrees(15))));
    controller1.rightBumper(skilledEventLoop).onTrue(new InstantCommand(() -> {
      unbindDrivetrainTeleop();
      moveArmCommand.schedule();
    }));
    controller1.rightBumper(skilledEventLoop).onFalse(new InstantCommand(() -> {
      bindDrivetrainTeleop();
      moveArmCommand.cancel();
    }));
  }
  private void configureSuperviseBindings(){
    controller1.a(superviseEventLoop).onTrue(new InstantCommand(aimToAprilTagCommand::reload));
    controller1.x(superviseEventLoop).onTrue(new InstantCommand(aimToAprilTagCommand::switchState));
  }

  public Command getAutonomousCommand() {
    AutoCommands.firstSpinUp = true;
    return autonomousCommand;
  }

  public Command getAutoShootCommand(){
      return Commands.either(
          arm.getPitchControlCommand(driveTrain).andThen(
                    new WaitUntilCommand(() -> Math.abs(shooter.getTopRPM()) > 4206.9 && arm.atSetpoint()),
                    new InstantCommand(intakeDriverCommand::buttonPress)),
            Commands.none(),
        intakeDriverCommand::readyToShoot
      ).andThen(new MoveArmToSetpointCommand(arm, Arm.SetpointOptions.INTAKE, ()->true)).andThen(toggleShooterCommand::stop);
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

  public void unbindDrivetrainTeleop() {
    Utilities.removeAndCancelDefaultCommand(driveTrain);
  }

  public void unBindDrivetrainTestMode() {
    Utilities.runIfNotNull(currentAdjustmentCommand, Command::cancel);
  }

  public void hapticsPeriodic() {
    hapticLoop.poll();
  }
}
