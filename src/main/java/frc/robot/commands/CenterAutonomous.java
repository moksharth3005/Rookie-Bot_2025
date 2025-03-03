// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.CANDriveSubsystem;
import frc.robot.subsystems.CoralShooter;

/* You should consider using the more terse Command factories API instead https://docs.wpilib.org/en/stable/docs/software/commandbased/organizing-command-based.html#defining-commands */
public class CenterAutonomous extends Command {
  /** Creates a new CenterAutonomous. */
  private ArcadeDrive leftEncoder = null;
  private ArcadeDrive rightEncoder = null;
  private CANDriveSubsystem leftMotor = null;
  private CANDriveSubsystem rightMotor = null;
  private CoralShooter Shooter = null;
  public CenterAutonomous(ArcadeDrive leftEncoder, ArcadeDrive rightEncoder, CANDriveSubsystem leftMotor, CANDriveSubsystem rightMotor, CoralShooter Shooter) {
    this.leftEncoder = leftEncoder;
    this.rightEncoder = rightEncoder;
    this.leftMotor = leftMotor;
    this.rightMotor = rightMotor; 
    this.Shooter = Shooter;
    }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {

  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    double Circumfearnce = 3.141592 * 3 * 2;
    double LeftDistanceTraveled = Circumfearnce * leftEncoder.Encoder_Leftleader.get();
    double RightDistanceTraveled = Circumfearnce * rightEncoder.Encoder_Rightleader.get();
    while (LeftDistanceTraveled <= 72 && RightDistanceTraveled <= 72){
      leftMotor.leftLeader.set(0.5);
      rightMotor.rightLeader.set(0.5);
      leftMotor.leftLeader.feed();
      rightMotor.rightLeader.feed();
    }
    if (LeftDistanceTraveled == 72 && RightDistanceTraveled == 72){
      CoralShooter.m_Shooter.set(0.3);
      CoralShooter.m_Shooter.feed();
      Timer.delay(1);
      CoralShooter.m_Shooter.set(0);
    }
    while (LeftDistanceTraveled >= 70 && RightDistanceTraveled >= 70){
      leftMotor.leftLeader.set(-0.5);
      rightMotor.rightLeader.set(-0.5);
      leftMotor.leftLeader.feed();
      rightMotor.rightLeader.feed();
    }
    while (LeftDistanceTraveled >= 58.22 && RightDistanceTraveled <= 81.78){
      leftMotor.leftLeader.set(-0.5);
      rightMotor.rightLeader.set(0.5);
      leftMotor.leftLeader.feed();
      rightMotor.rightLeader.feed();
    }
    

  }
    
  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
