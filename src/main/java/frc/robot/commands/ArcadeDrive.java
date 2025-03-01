// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj.DutyCycleEncoder;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.CANDriveSubsystem;

/* You should consider using the more terse Command factories API instead https://docs.wpilib.org/en/stable/docs/software/commandbased/organizing-command-based.html#defining-commands */
public class ArcadeDrive extends Command {
  private CANDriveSubsystem driveTrain = null;
  private XboxController controller = null;
  private DutyCycleEncoder Encoder_Rightleader = null;
  private DutyCycleEncoder Encoder_Leftleader = null;

  /** Creates a new ArcadeDrive. */
  public ArcadeDrive(CANDriveSubsystem driveTrain, XboxController controller, DutyCycleEncoder Encoder_Rightleader, DutyCycleEncoder Encoder_Leftleader) {
    this.driveTrain = driveTrain;
    this.controller = controller;
    this.Encoder_Leftleader = Encoder_Leftleader;
    this.Encoder_Rightleader = Encoder_Rightleader;


    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(driveTrain);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    Encoder_Leftleader.get();
    Encoder_Rightleader.get();
    if (controller.getLeftY() <= 0.04 && controller.getLeftY() >= -0.04){ //No y input
      driveTrain.leftLeader.set(controller.getRightX() >= 0.04 ? controller.getRightX() : 0);
      if (controller.getRightX() > 0.04){ // Going Left
      driveTrain.leftLeader.set(controller.getRightX() * -1);
      driveTrain.rightLeader.set(controller.getRightX());
      driveTrain.leftLeader.set(controller.getRightX() >= 0.04 ? controller.getRightX() : 0);
      driveTrain.leftLeader.feed();
      driveTrain.rightLeader.feed();
      } else if (controller.getRightX() < 0.04){ // Going Right
        driveTrain.leftLeader.set(controller.getRightX());
        driveTrain.rightLeader.set(controller.getRightX() * -1);
        driveTrain.leftLeader.set(controller.getRightX() >= 0.04 ? controller.getRightX() : 0);
        driveTrain.leftLeader.feed();
        driveTrain.rightLeader.feed();
      }
    } else if (controller.getRightX() == 0.04){ // No Right/Left
      driveTrain.leftLeader.set(controller.getLeftY() * -1);
      driveTrain.rightLeader.set(controller.getLeftY() * -1);
      driveTrain.leftLeader.feed();
      driveTrain.rightLeader.feed();
    } else if (controller.getLeftY() < 0.04 && controller.getRightX() > 0.04){ // Forward and Right
      double Ymed = Math.abs(controller.getLeftY())/0.4;
      double Yfin = controller.getLeftY() - Ymed;
      System.out.println(Yfin);
      driveTrain.rightLeader.set((controller.getRightX() * -1) + Yfin);
      driveTrain.leftLeader.set(controller.getRightX());
      driveTrain.leftLeader.set(controller.getRightX() >= 0.04 ? controller.getRightX() : 0);
      driveTrain.leftLeader.feed();
      driveTrain.rightLeader.feed();
    } else if (controller.getLeftY() > 0.04 && controller.getRightX() > 0.04){ // Backward and right
      double Ymed = Math.abs(controller.getLeftY())/0.4;
      double Yfin = controller.getLeftY() - Ymed;  
      System.out.println(Yfin);   
      driveTrain.leftLeader.set(controller.getRightX() + Yfin);
      driveTrain.rightLeader.set(controller.getRightX() * -1);
      driveTrain.leftLeader.set(controller.getRightX() >= 0.04 ? controller.getRightX() : 0);
      driveTrain.leftLeader.feed();
      driveTrain.rightLeader.feed();
    } else if (controller.getLeftY() < 0.04 && controller.getLeftX() < 0.04){ //Forward and left
      double Ymed = Math.abs(controller.getLeftY())/0.4;
      double Yfin = controller.getLeftY() - Ymed;
      System.out.println(Yfin);     
      driveTrain.rightLeader.set((controller.getRightX()) + Yfin);
      driveTrain.leftLeader.set(controller.getRightX() * -1);
      driveTrain.leftLeader.set(controller.getRightX() >= 0.04 ? controller.getRightX() : 0);
      driveTrain.leftLeader.feed();
      driveTrain.rightLeader.feed();
    } else if (controller.getLeftY() > 0.04 && controller.getRightX() < 0.04){ //Backward and Left
      double Ymed = Math.abs(controller.getLeftY())/0.4;
      double Yfin = controller.getLeftY() - Ymed;
      System.out.println(Yfin);     
      driveTrain.leftLeader.set((controller.getRightX() * -1) + Yfin);
      driveTrain.rightLeader.set(controller.getRightX());
      driveTrain.leftLeader.set(controller.getRightX() >= 0.04 ? controller.getRightX() : 0);
      driveTrain.leftLeader.feed();
      driveTrain.rightLeader.feed();
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
