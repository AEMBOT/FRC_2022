package frc.robot.commands.climber;

public class ClimbLidar {

  /*

  private boolean reachedTarget() {
    // Let's assuming linear slowdown for now. Bernoull's equation is technically
    // squared in velocity. But for small velocities, maybe it's linear.
    //

    if (climber.getCurrentState() == ClimberState.kRetracting) {
      return (climber.getCurrentDistance() + .5 * climber.getCurrentVelocity() * Constants.RetractionAfterglowSeconds < Constants.RetractionTargetDistanceMM);
    } else if (climber.getCurrentState() == ClimberState.kExtending) {
      return (climber.getCurrentDistance() + .5 * climber.getCurrentVelocity() * Constants.ExtensionAfterglowSeconds > Constants.ExtensionTargetDistanceMM);
    } else if (climber.getCurrentState() == ClimberState.kCoasting) {
      // Only advance to next state if the hooks are in the right spot!
      return (climber.getCurrentDistance() < Constants.RetractionTargetDistanceMM ||
              climber.getCurrentDistance() > Constants.ExtensionTargetDistanceMM);
    } else {
      // Shouldn't be here! But return false just to be safe
      return false;
    }
  }
  */
}
