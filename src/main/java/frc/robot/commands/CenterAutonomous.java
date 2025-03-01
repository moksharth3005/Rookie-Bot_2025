// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;

/* You should consider using the more terse Command factories API instead https://docs.wpilib.org/en/stable/docs/software/commandbased/organizing-command-based.html#defining-commands */
public class CenterAutonomous extends Command {
  /** Creates a new CenterAutonomous. */
  private ArcadeDrive LeftEncoder = null;
  private ArcadeDrive rightEncoder = null;
  public CenterAutonomous(ArcadeDrive leftEncoder, ArcadeDrive rightEncoder) {
    this.LeftEncoder = LeftEncoder;
    this.rightEncoder = rightEncoder;
    // Use addRequirements() here to declare subsystem dependencies.
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {}
  private double distanceTraveled = 3.141592 * Math.pow(3, 2);
    
  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
