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
import frc.robot.commands.IntakeCommands;
import frc.robot.commands.MoveToReef;
import frc.robot.commands.MultiCommands;
import frc.robot.commands.ShoulderCommands;
import frc.robot.generated.TunerConstants;
import frc.robot.subsystems.drive.Drive;
import frc.robot.subsystems.drive.GyroIO;
import frc.robot.subsystems.drive.GyroIOPigeon2;
import frc.robot.subsystems.drive.ModuleIO;
import frc.robot.subsystems.drive.ModuleIOSim;
import frc.robot.subsystems.drive.ModuleIOTalons;
import frc.robot.subsystems.elevator.Elevator;
import frc.robot.subsystems.elevator.ElevatorConstants;
import frc.robot.subsystems.elevator.ElevatorIOSpark;
import frc.robot.subsystems.intake.Intake;
import frc.robot.subsystems.intake.IntakeConstants;
import frc.robot.subsystems.intake.IntakeIOSpark;
import frc.robot.subsystems.shoulder.Shoulder;
import frc.robot.subsystems.shoulder.ShoulderIOSpark;
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
  private final Shoulder shoulder;

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
    elevator = new Elevator(new ElevatorIOSpark());
    shoulder = new Shoulder(new ShoulderIOSpark());
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

    NamedCommands.registerCommand("AlignToReefTag", new AlignToReef(drive, vision));

    NamedCommands.registerCommand("MoveToReef", new MoveToReef(drive, vision));

    // Configure the button bindings
    configureButtonBindings();
    configureSubsystemLogic();
  }

  private void configureSubsystemLogic() {
    Trigger elevatorLimitSwtichTrigger = new Trigger(() -> elevator.eLimitSwitch());

    // if the shoulder is down, and the ACTUAL HEIGHT of the elevator is too low, then we need to
    // move the shoulder up.
    Trigger shoulderCrashTrigger =
        new Trigger(
            () ->
                shoulder.getAngle() < 1 && elevator.getHeight() < ElevatorConstants.shoulderLength);

    // If the REQUESTED HEIGHT of the elevator is lower than a height where it would hit the intake,
    // then we need to move the intake out of the way (if its ACTUALLY IN)
    Trigger elevatorLoweringTrigger =
        new Trigger(
            () ->
                elevator.getRHeight() < ElevatorConstants.intakeHeight && !intake.getIntakeOpen());

    // If the REQUESTED ANGLE of the intake is in, and the ACTUAL HEIGHT of the elevator is too low,
    // then we need to move the elevator up out of the way.
    Trigger elevatorUpTrigger =
        new Trigger(
            () ->
                elevator.getHeight() < ElevatorConstants.intakeHeight
                    && !intake.getRIntakeOpen()); // for shoulder UP cases

    shoulderCrashTrigger.onTrue(
        new InstantCommand(
            () -> shoulder.setRAngle(0.25),
            shoulder)); // move the shoulder straight up to avoid crashing into the robot.
    elevatorLoweringTrigger.onTrue(
        new InstantCommand(
            () -> intake.setIntakeAngle(IntakeConstants.intakeStartUpAngle + 1),
            intake)); // Move the intake to a safe position.
    elevatorUpTrigger.onTrue(
        new InstantCommand(
            () -> elevator.setRHeight(ElevatorConstants.intakeHeight + 2),
            elevator)); // Move the elevator up out of the way.
    elevatorLimitSwtichTrigger.onTrue(
        new InstantCommand(elevator::resetEncoder)); // reset the encoder.
  }

  private void configureButtonBindings() {
    // Default command, normal field-relative drive
    drive.setDefaultCommand(
        DriveCommands.joystickDrive(
            drive,
            () -> -controller.getLeftY(),
            () -> -controller.getLeftX(),
            () -> -controller.getRightX()));

    // Run intake routine when Y button is pressed
    controller
        .y()
        .whileTrue(
            new AlignToCoral(
                drive, vision, 2, () -> -controller.getLeftY(), () -> -controller.getLeftX()));
    // Intake and handoff on bumper press.
    fightBox.button(3).onTrue(ElevatorCommands.goToTier(elevator, 1));
    fightBox.button(4).onTrue(ElevatorCommands.goToTier(elevator, 2));
    fightBox.button(6).onTrue(ElevatorCommands.goToTier(elevator, 3));
    fightBox.button(5).onTrue(ElevatorCommands.goToTier(elevator, 4));
    fightBox.pov(0).onTrue(ShoulderCommands.place(shoulder));
    fightBox.pov(90).onTrue(ShoulderCommands.moveShoulderTo(shoulder, 0.75));

    // controller.leftBumper().onTrue (IntakeCommands.runIntakeRoutine(intake));
    // Score on right bumper

    controller.a().onTrue(IntakeCommands.scoreL1(intake));
    controller.rightBumper().onTrue(MultiCommands.handOff(intake, elevator, shoulder));
    controller.leftBumper().onTrue(IntakeCommands.intakeCoralAndStow(intake));

    controller.rightTrigger().whileTrue(Commands.run(() -> intake.adjustRotatorAngle(0.5), intake));
    controller.leftTrigger().whileTrue(Commands.run(() -> intake.adjustRotatorAngle(-0.5), intake));

    controller.pov(0).whileTrue(Commands.run(() -> elevator.adjustRHeight(0.5), elevator));
    controller.pov(180).whileTrue(Commands.run(() -> elevator.adjustRHeight(-0.5), elevator));
    controller.pov(90).whileTrue(Commands.run(() -> shoulder.adjustRAngle(0.008), shoulder));
    controller.pov(270).whileTrue(Commands.run(() -> shoulder.adjustRAngle(-0.008), shoulder));

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
