// Copyright 2021-2024 FRC 6328
// http://github.com/Mechanical-Advantage
//
// This program is free software; you can redistribute it and/or
// modify it under the terms of the GNU General Public License
// version 3 as published by the Free Software Foundation or
// available in the root directory of this project.
//
// This program is distributed in the hope that it will be useful,
// but WITHOUT ANY WARRANTY; without even the implied warranty of
// MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
// GNU General Public License for more details.
// github push test

package frc.robot;

import static frc.robot.subsystems.vision.VisionConstants.*;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.auto.NamedCommands;
import com.pathplanner.lib.commands.PathPlannerAuto;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.button.CommandGenericHID;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.commands.AlignToCoral;
import frc.robot.commands.AlignToReef;
import frc.robot.commands.DriveCommands;
import frc.robot.commands.ElevatorCommands;
import frc.robot.commands.IntakeCommandClosedLoop;
import frc.robot.commands.IntakeCommands;
import frc.robot.commands.MoveToReef;
import frc.robot.generated.TunerConstants;
import frc.robot.subsystems.drive.Drive;
import frc.robot.subsystems.drive.GyroIO;
import frc.robot.subsystems.drive.GyroIOPigeon2;
import frc.robot.subsystems.drive.ModuleIO;
import frc.robot.subsystems.drive.ModuleIOSim;
import frc.robot.subsystems.drive.ModuleIOTalons;
import frc.robot.subsystems.elevator.Elevator;
import frc.robot.subsystems.elevator.ElevatorIOSpark;
import frc.robot.subsystems.intake.Intake;
import frc.robot.subsystems.intake.IntakeConstants;
import frc.robot.subsystems.intake.IntakeIOSpark;
import frc.robot.subsystems.vision.Vision;
import frc.robot.subsystems.vision.VisionIO;
import frc.robot.subsystems.vision.VisionIOPhotonVision;
import frc.robot.subsystems.vision.VisionIOPhotonVisionSim;
import frc.robot.subsystems.vision.VisionIOTargetOnly;
import org.littletonrobotics.junction.networktables.LoggedDashboardChooser;

/**
 * This class is where the bulk of the robot should be declared. Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of the robot (including
 * subsystems, commands, and button mappings) should be declared here.
 */
public class RobotContainer {
  // Subsystems
  private final Drive drive;
  private final Vision vision;
  private final Intake intake;
  private final Elevator elevator;
  private Trigger eLimitSwitchTrigger;

  // Controller
  private final CommandXboxController controller = new CommandXboxController(0);
  private final CommandGenericHID fightBox = new CommandGenericHID(1);

  // Dashboard inputs
  private final LoggedDashboardChooser<Command> autoChooser;

  /** The container for the robot. Contains subsystems, OI devices, and commands. */
  public RobotContainer() {
    switch (Constants.currentMode) {
      case REAL:
        // Real robot, instantiate hardware IO implementations
        drive =
            new Drive(
                new GyroIOPigeon2(),
                new ModuleIOTalons(TunerConstants.FrontLeft),
                new ModuleIOTalons(TunerConstants.FrontRight),
                new ModuleIOTalons(TunerConstants.BackLeft),
                new ModuleIOTalons(TunerConstants.BackRight));
        vision =
            new Vision(
                drive,
                drive::addVisionMeasurement,
                new VisionIOPhotonVision(camera1Name, robotToCamera1),
                new VisionIOTargetOnly(camera2Name));
        break;

      case SIM:
        // Sim robot, instantiate physics sim IO implementations
        drive =
            new Drive(
                new GyroIO() {},
                new ModuleIOSim(TunerConstants.FrontLeft),
                new ModuleIOSim(TunerConstants.FrontRight),
                new ModuleIOSim(TunerConstants.BackLeft),
                new ModuleIOSim(TunerConstants.BackRight));

        vision =
            new Vision(
                drive,
                drive::addVisionMeasurement,
                new VisionIOPhotonVisionSim(camera1Name, robotToCamera1, drive::getPose),
                new VisionIOPhotonVisionSim(camera2Name, robotToCamera2, drive::getPose));
        break;

      case TEST: // Make a "blank" drive and "blank" vision, as these subsystems are most expensive
        // and not needed for testing.
        drive =
            new Drive(
                new GyroIO() {},
                new ModuleIO() {},
                new ModuleIO() {},
                new ModuleIO() {},
                new ModuleIO() {});

        vision =
            new Vision(drive, drive::addVisionMeasurement, new VisionIO() {}, new VisionIO() {});
        break;

      default:
        // Replayed robot, disable IO implementations
        drive =
            new Drive(
                new GyroIO() {},
                new ModuleIO() {},
                new ModuleIO() {},
                new ModuleIO() {},
                new ModuleIO() {});

        vision =
            new Vision(drive, drive::addVisionMeasurement, new VisionIO() {}, new VisionIO() {});
        break;
    }

    intake = new Intake(new IntakeIOSpark());
    elevator = new Elevator(new ElevatorIOSpark(), intake::getIntakeOpen);

    // Set up auto routines
    autoChooser = new LoggedDashboardChooser<>("Auto Choices", AutoBuilder.buildAutoChooser());
    /*
    // Set up SysId routines
    autoChooser.addOption(
        "Drive Wheel Radius Characterization", DriveCommands.wheelRadiusCharacterization(drive));
    autoChooser.addOption(
        "Drive Simple FF Characterization", DriveCommands.feedforwardCharacterization(drive));
    autoChooser.addOption(
        "Drive SysId (Quasistatic Forward)",
        drive.sysIdQuasistatic(SysIdRoutine.Direction.kForward));
    autoChooser.addOption(
        "Drive SysId (Quasistatic Reverse)",
        drive.sysIdQuasistatic(SysIdRoutine.Direction.kReverse));
    autoChooser.addOption(
        "Drive SysId (Dynamic Forward)", drive.sysIdDynamic(SysIdRoutine.Direction.kForward));
    autoChooser.addOption(
        "Drive SysId (Dynamic Reverse)", drive.sysIdDynamic(SysIdRoutine.Direction.kReverse));
    */
    autoChooser.addDefaultOption("Taxi back", new PathPlannerAuto("LeaveAuto"));
    autoChooser.addOption("Middle Auto", new PathPlannerAuto("MiddleAuto"));

    NamedCommands.registerCommand(
        "PlaceOnTier4", ElevatorCommands.placeOnTier(4, elevator, intake));
    
    NamedCommands.registerCommand(
        "AlignToReefTag", new AlignToReef(drive, vision));
    
    NamedCommands.registerCommand(
        "MoveToReef", new MoveToReef(drive, vision));
    
    eLimitSwitchTrigger = new Trigger(elevator.eLimitSwitch());
    eLimitSwitchTrigger.onTrue(new InstantCommand(elevator::resetEncoder));
    // Configure the button bindings
    configureButtonBindings();
  }

  /**
   * Use this method to define your button->command mappings. Buttons can be created by
   * instantiating a {@link GenericHID} or one of its subclasses ({@link
   * edu.wpi.first.wpilibj.Joystick} or {@link XboxController}), and then passing it to a {@link
   * edu.wpi.first.wpilibj2.command.button.JoystickButton}.
   */
  private void configureButtonBindings() {
    // Default command, normal field-relative drive
    drive.setDefaultCommand(
        DriveCommands.joystickDrive(
            drive,
            () -> -controller.getLeftY(),
            () -> -controller.getLeftX(),
            () -> -controller.getRightX()));

    // Lock to 0° when A button is held
    controller
        .a()
        .whileTrue(
            DriveCommands.joystickDriveAtAngle(
                drive,
                () -> -controller.getLeftY(),
                () -> -controller.getLeftX(),
                () -> new Rotation2d()));

    // Run intake routine when Y button is pressed
    controller
        .y()
        .whileTrue(
            new AlignToCoral(
                drive, vision, 2, () -> -controller.getLeftY(), () -> -controller.getLeftX()));
    // Intake and handoff on bumper press.
    fightBox.button(3).onTrue(ElevatorCommands.goToTier(1, elevator, intake));
    fightBox.button(4).onTrue(ElevatorCommands.goToTier(2, elevator, intake));
    fightBox.button(6).onTrue(ElevatorCommands.goToTier(3, elevator, intake));
    fightBox.button(5).onTrue(ElevatorCommands.goToTier(4, elevator, intake));
    fightBox.pov(0).onTrue(ElevatorCommands.place(elevator));
    fightBox
        .pov(90)
        .onTrue(
            new InstantCommand(
                () -> {
                  elevator.setRAngle(0.75);
                },
                elevator));

    // controller.leftBumper().onTrue (IntakeCommands.runIntakeRoutine(intake));
    // Score on right bumper
    controller.leftBumper().onTrue(IntakeCommandClosedLoop.rotateTo(intake, IntakeConstants.intakeLevelOneAngle));
    controller.rightBumper().onTrue(IntakeCommandClosedLoop.rotateTo(intake, IntakeConstants.intakeIntakeAngle));
    // Reset gyro to 0° when B button is pressed

    controller.pov(0).whileTrue(Commands.run(() -> elevator.adjustRHeight(0.5), elevator));
    controller.pov(180).whileTrue(Commands.run(() -> elevator.adjustRHeight(-0.5), elevator));
    controller.pov(90).whileTrue(Commands.run(() -> elevator.adjustRAngle(0.008), elevator));
    controller.pov(270).whileTrue(Commands.run(() -> elevator.adjustRAngle(-0.008), elevator));

    controller
        .b()
        .onTrue(
            Commands.runOnce(
                    () ->
                        drive.setPose(
                            new Pose2d(drive.getPose().getTranslation(), new Rotation2d())),
                    drive)
                .ignoringDisable(true));
  }

  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand() {
    return new PathPlannerAuto("LeaveAuto");
  }
}
