// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.Servo;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.Constants.ShooterConstants;
import frc.robot.subsystems.DriveSubsystem;

public class ShooterHood extends SubsystemBase {

  private Servo shooterHoodServoRight;
  private Servo shooterHoodServoLeft;

  /** Creates a new ShooterHood. */
  public ShooterHood() {
    shooterHoodServoRight = new Servo(
      Constants.ShooterConstants.kShooterHoodServoRightChannel);

    shooterHoodServoLeft = new Servo(
      Constants.ShooterConstants.kShooterHoodServoLeftChannel);

    shooterHoodServoLeft.setBoundsMicroseconds(
      2565,
    2564,
    1882,
    1201, 
    1200   
    );

    shooterHoodServoRight.setBoundsMicroseconds(
      2565,
    2564,
    1882,
    1201, 
    1200   
    );

    shooterHoodServoRight.setAngle(0);
    shooterHoodServoLeft.setAngle(0);
  }

  public void setHoodAngle(double length) {
    // if (length > ShooterConstants.kShooterMaxAngleHoodmm) {
    //   length = ShooterConstants.kShooterMaxAngleHoodmm;
    // } else if (length < ShooterConstants.kShooterMinAngleHoodmm) {
    //   length = ShooterConstants.kShooterMinAngleHoodmm;
    // }
    shooterHoodServoRight.setAngle(length);
    shooterHoodServoLeft.setAngle(length);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
