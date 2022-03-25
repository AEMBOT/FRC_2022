package frc.robot.commands.intake;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.IntakeSubsystem;
import java.util.function.DoubleSupplier;

public class ManualIntakeLift extends CommandBase {
  private IntakeSubsystem m_intake;
  private DoubleSupplier m_power;

  public ManualIntakeLift(IntakeSubsystem intake, DoubleSupplier control) {
    m_intake = intake;
    m_power = control;
    addRequirements(m_intake);
  }

  @Override
  public void execute() {
    double power = MathUtil.applyDeadband(m_power.getAsDouble(), 0.02) * 0.2;
    m_intake.setLiftPower(power);
    SmartDashboard.putNumber("Lift Power", power);
  }
}
