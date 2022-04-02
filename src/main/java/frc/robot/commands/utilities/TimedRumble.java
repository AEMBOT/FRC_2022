package frc.robot.commands.utilities;

import edu.wpi.first.wpilibj.GenericHID.RumbleType;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.CommandBase;

/** A command that rumbles a controller for a specific amount of time. */
public class TimedRumble extends CommandBase {
  // A timer to keep track of how long the controller has been rumbling
  private final Timer m_timer = new Timer();

  private XboxController m_controller;
  private double m_duration;
  private double m_strength;

  /**
   * Rumbles a controller for the specified amount of time.
   *
   * @param controller The {@link XboxController} to vibrate
   * @param seconds The duration to vibrate the controller for
   * @param strength Strength of vibration (between 0 and 1)
   */
  public TimedRumble(XboxController controller, double seconds, double strength) {
    m_controller = controller;
    m_duration = seconds;
    m_strength = strength;
  }

  @Override
  public void initialize() {
    m_timer.start();
  }

  @Override
  public void execute() {
    // Rumble both sides of the controller at the specified strength
    m_controller.setRumble(RumbleType.kLeftRumble, m_strength);
    m_controller.setRumble(RumbleType.kRightRumble, m_strength);
  }

  @Override
  public void end(boolean _interrupted) {
    // Stop rumbling the controller
    m_controller.setRumble(RumbleType.kLeftRumble, 0);
    m_controller.setRumble(RumbleType.kRightRumble, 0);

    // Stop the timer
    m_timer.stop();

    // Reset the timer in case this command is reused
    m_timer.reset();
  }

  @Override
  public boolean isFinished() {
    // Check if the specified duration has elapsed
    return m_timer.get() >= m_duration;
  }
}
