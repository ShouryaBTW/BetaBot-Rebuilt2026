package frc.robot;

import java.util.Set;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import edu.wpi.first.wpilibj2.command.WaitUntilCommand;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.Constants.OIConstants;
import frc.robot.commands.AimToTag;
import frc.robot.commands.DriveToPoint;
import frc.robot.commands.ResetPose;
import frc.robot.commands.ShootSequence;
import frc.robot.commands.StartFeeder;
import frc.robot.commands.StartFloor;
import frc.robot.commands.StartShooter;
import frc.robot.commands.TurnToAngle;
import frc.robot.subsystems.DriveSubsystem;
import frc.robot.subsystems.FeederSubsystem;
import frc.robot.subsystems.FloorSubsystem;
import frc.robot.subsystems.LimelightSubsystem;
import frc.robot.subsystems.ShooterSubsystem;
/*
 * This class is where the bulk of the robot should be declared.  Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls).  Instead, the structure of the robot
 * (including subsystems, commands, and button mappings) should be declared here.
 */
public class RobotContainer {
  // The robot's subsystems
  private final DriveSubsystem m_robotDrive = new DriveSubsystem();
  private final ShooterSubsystem s_shooterSubsystem = new ShooterSubsystem();
  private final FloorSubsystem s_floorSubsystem = new FloorSubsystem();
  private final FeederSubsystem s_feederSubsystem = new FeederSubsystem();
  private final LimelightSubsystem s_limelightSubsystem = new LimelightSubsystem();

  // The driver's controller
  XboxController m_driverController = new XboxController(OIConstants.kDriverControllerPort);

  /**
   * The container for the robot. Contains subsystems, OI devices, and commands.
   */
  public RobotContainer() {
    // Configure the button bindings
    configureButtonBindings();

    // Configure default commands
    m_robotDrive.setDefaultCommand(
        // The left stick controls translation of the robot.
        // Turning is controlled by the X axis of the right stick.
        new RunCommand(
            () -> m_robotDrive.drive(
                -MathUtil.applyDeadband(m_driverController.getLeftY(), OIConstants.kDriveDeadband),
                -MathUtil.applyDeadband(m_driverController.getLeftX(), OIConstants.kDriveDeadband),
                -MathUtil.applyDeadband(m_driverController.getRightX(), OIConstants.kDriveDeadband),
                true),
            m_robotDrive));
  }

  /**
   * Use this method to define your button->command mappings. Buttons can be
   * created by
   * instantiating a {@link edu.wpi.first.wpilibj.GenericHID} or one of its
   * subclasses ({@link
   * edu.wpi.first.wpilibj.Joystick} or {@link XboxController}), and then calling
   * passing it to a
   * {@link JoystickButton}.
   */
  private void configureButtonBindings() {
    // Final Joystick Commands
    new JoystickButton(m_driverController, XboxController.Button.kStart.value)
        .whileTrue(new InstantCommand(() -> m_robotDrive.zeroHeading(), m_robotDrive));

    // Auto Test
    new JoystickButton(m_driverController, XboxController.Button.kA.value)
      .whileTrue(new SequentialCommandGroup(
        new ResetPose(m_robotDrive, s_limelightSubsystem),
        new DriveToPoint(m_robotDrive, 1.89, 0, 90),
        new DriveToPoint(m_robotDrive, 1.89, 1.95, 90),
        new DriveToPoint(m_robotDrive, 1.89, 0, 180),
        new ResetPose(m_robotDrive, s_limelightSubsystem),
        new DriveToPoint(m_robotDrive, -1.85, 0, 65),
        new ShootSequence(s_shooterSubsystem, s_feederSubsystem, s_floorSubsystem, m_robotDrive, s_limelightSubsystem).withTimeout(6),
        new TurnToAngle(m_robotDrive, 180),
        new WaitCommand(.20),
        new ResetPose(m_robotDrive, s_limelightSubsystem),
        new WaitCommand(.20),
        new DriveToPoint(m_robotDrive, 0.81, 0, 180),
        new WaitCommand(5),
        new DriveToPoint(m_robotDrive, 2.81, 0, 70),
        new ShootSequence(s_shooterSubsystem, s_feederSubsystem, s_floorSubsystem, m_robotDrive, s_limelightSubsystem).withTimeout(5)
      ));

    new JoystickButton(m_driverController, XboxController.Button.kX.value)
      .whileTrue(new TurnToAngle(m_robotDrive, s_limelightSubsystem, 2));

    // Reset Pose
    new JoystickButton(m_driverController, XboxController.Button.kY.value)
      .whileTrue(new ResetPose(m_robotDrive, s_limelightSubsystem));

    new JoystickButton(m_driverController, XboxController.Button.kRightBumper.value)
      .whileTrue(new ShootSequence(s_shooterSubsystem, s_feederSubsystem, s_floorSubsystem, m_robotDrive, s_limelightSubsystem));

    new JoystickButton(m_driverController, XboxController.Button.kLeftBumper.value) 
      .whileTrue(new StartShooter(s_shooterSubsystem, s_limelightSubsystem));
    }

  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand() {
    return null;
    // return new SequentialCommandGroup(
    //     new DriveToPoint(m_robotDrive, 3, 3, 270),
    //     new WaitCommand(1),
    //     new DriveToPoint(m_robotDrive, 0, 0, 270),
    //     new TurnToAngle(m_robotDrive, s_limelightSubsystem),
    //     new ShootSequence(s_shooterSubsystem, s_feederSubsystem, s_floorSubsystem, m_robotDrive, s_limelightSubsystem)
    // );
  }

//     public Command aimToAllowedTagOnce() {
//     return Commands.defer(
//         () -> {
//           // Gate: must see a target and it must be an allowed tag
//           if (!s_limelightSubsystem.hasValidTarget() || !s_limelightSubsystem.isTagAllowedForRotation()) {
//             return Commands.none();
//           }

//           double txDeg = s_limelightSubsystem.getTX();

//           // Use continuous heading for control math (not the wrapped [0..360) one)
//           double currentYawDeg = m_robotDrive.getPoseContinuous().getAngle();

//           // Typical mapping: targetYaw = currentYaw - tx
//           // If it turns the wrong way on your robot, flip the sign to +tx.
//           double targetYawDeg = currentYawDeg - txDeg;

//           return new TurnToAngle(m_robotDrive, targetYawDeg);
//         },
//         Set.of(m_robotDrive) // requirements for the *returned* command
//     );
//   }

public DriveSubsystem getDriveSubsystem() {
    return m_robotDrive;
  }
}