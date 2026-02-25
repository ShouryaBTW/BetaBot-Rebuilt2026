package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitUntilCommand;
import frc.robot.subsystems.DriveSubsystem;
import frc.robot.subsystems.FeederSubsystem;
import frc.robot.subsystems.FloorSubsystem;
import frc.robot.subsystems.LimelightSubsystem;
import frc.robot.subsystems.ShooterSubsystem;

public class ShootSequence extends SequentialCommandGroup {
  public ShootSequence(
      ShooterSubsystem shooter,
      FeederSubsystem feeder,
      FloorSubsystem floor,
      DriveSubsystem drive,
      LimelightSubsystem limelight
  ) {
    addCommands(

      new TurnToAngle(drive, limelight),

      new ParallelCommandGroup(
        // Lock drivetrain in place
        new RunCommand(() -> drive.setX(), drive),

        new ParallelCommandGroup(
          // Shooter runs the entire time, never interrupted
          new StartShooter(shooter, limelight),

          new SequentialCommandGroup(
            // Wait until shooter is up to speed before feeding
            new WaitUntilCommand(() -> shooter.atTargetSpeed(2.0)),

            // Then run floor and feeder
            new ParallelCommandGroup(
              new StartFloor(floor, 25),
              new StartFeeder(feeder, 50)
            )
          )
        )
      )
    );
  }
}