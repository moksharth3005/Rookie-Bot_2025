// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import frc.robot.Constants.DriveConstants;
import frc.robot.commands.DriveCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import com.ctre.phoenix.*;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;

public class CoralShooter extends SubsystemBase {
  public static WPI_TalonSRX m_Shooter;
  /** Creates a new CoralShooter. */
  public CoralShooter() {
    m_Shooter = new WPI_TalonSRX(DriveConstants.SHOOTER);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
