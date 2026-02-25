// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.FeederSubsystem;

public class StartFeeder extends Command {
  public final FeederSubsystem feederSubsystem;
  public final double feederTargetRPS;

  public StartFeeder(FeederSubsystem feederSubsystem, double feederTargetRPS) {
    this.feederSubsystem = feederSubsystem;
    this.feederTargetRPS = feederTargetRPS;
  }

  @Override
  public void initialize() {}

  @Override
  public void execute() {
    feederSubsystem.setVelocity(feederTargetRPS);
  }

  @Override
  public void end(boolean interrupted) {
    feederSubsystem.stop();
  }

  @Override
  public boolean isFinished() {
    return false;
  }
}
