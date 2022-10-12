// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import static frc.robot.Constants.ClimberConstants.*;

import edu.wpi.first.wpilibj.PneumaticsModuleType;
import edu.wpi.first.wpilibj.Solenoid;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.hardware.ClimberArm;

/** A subsystem that controls the pistons involved in Tupperware's climber. */
public class ClimberSubsystem extends SubsystemBase {
  // The two climber arms (i.e. the large pistons & their cutoffs)
  private final ClimberArm m_leftArm =
      new ClimberArm(kClimbSolenoidLeftExtend, kClimbSolenoidLeftRetract, kClimbSolenoidLeftChoke);
  private final ClimberArm m_rightArm =
      new ClimberArm(
          kClimbSolenoidRightExtend, kClimbSolenoidRightRetract, kClimbSolenoidRightChoke);

  public ClimberSubsystem() {
    super();
    this.retractArms();
    this.setPistonsVertical();
  }  

  // The solenoid controlling the pistons that angle the climber arms
  private final Solenoid m_anglePistons =
      new Solenoid(PneumaticsModuleType.CTREPCM, kAngleSolenoid);

  /** Extends both climber pistons. */
  public void extendArms() {
    m_leftArm.extendArm();
    m_rightArm.extendArm();
  }

  /** Retracts both climber pistons. */
  public void retractArms() {
    m_leftArm.retractArm();
    m_rightArm.retractArm();
  }

  /** Cuts off pressure to both climber pistons. */
  public void cutOffPressure() {
    m_leftArm.cutOffPressure();
    m_rightArm.cutOffPressure();
  }

  /** Returns the climber pistons to a vertical position. */
  public void setPistonsVertical() {
    m_anglePistons.set(false);
  }

  /** Angles the climber pistons. */
  public void setPistonsAngled() {
    m_anglePistons.set(true);
  }
}
