package frc.robot.commands;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.launcher.Launcher;
import frc.robot.subsystems.launcher.Launcher.LauncherState;

public class AutoSpeaker extends Command {

  private Launcher launcher;

  private boolean ended;

  private double startTime;
  private double elapsedTime;
  private double windup = 1.0;
  private double duration = windup + .5;

  public AutoSpeaker() {
    launcher = Launcher.getInstance();
  }

  @Override
  public void initialize() {
    ended = false;

    launcher.setLauncherState(LauncherState.SPEAKER);
    startTime = -1;
    elapsedTime = 0;
    launcher.setLauncherOn();
  }

  @Override
  public void execute() {
    launcher.updatePose();

    SmartDashboard.putNumber("Start Time", startTime);
    SmartDashboard.putNumber("Elapsed Time", elapsedTime);

    if(launcher.hasReachedPose(2.0)){
      if (startTime == -1) {
        startTime = Timer.getFPGATimestamp();
      }
      elapsedTime = Timer.getFPGATimestamp() - startTime;
      System.out.println("anshanshanshanshansh");

      if (elapsedTime > windup) {
        launcher.setFlickerOn();
      }

      if (elapsedTime > duration) {
        ended = true;
      }
    }
  }

  @Override
  public void end(boolean interrupted) {
    launcher.setFlickOff();
    launcher.setLauncherOff();
  }

  @Override
  public boolean isFinished() {
    return ended;
  }
}
