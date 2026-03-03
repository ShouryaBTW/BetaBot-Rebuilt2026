package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.IntakeSubsystem;

public class StartIntake extends Command {
  public final IntakeSubsystem intakeSubsystem;
  public final double intakeTargetRPS;

  public StartIntake(IntakeSubsystem intakeSubsystem, double intakeTargetRPS) {
    this.intakeSubsystem = intakeSubsystem;
    this.intakeTargetRPS = intakeTargetRPS;

    addRequirements(intakeSubsystem);
  }

  @Override
  public void initialize() {}

  @Override
  public void execute() {
    intakeSubsystem.setRollerVelocity(intakeTargetRPS);
  }

  @Override
  public void end(boolean interrupted) {
    intakeSubsystem.stopRoller();
  }

  @Override
  public boolean isFinished() {
    return false;
  }
}
