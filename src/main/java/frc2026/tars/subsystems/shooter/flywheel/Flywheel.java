package frc2026.tars.subsystems.shooter.flywheel;

import com.ctre.phoenix6.controls.VelocityTorqueCurrentFOC;
import com.teamscreamrobotics.drivers.TalonFXSubsystem;
import edu.wpi.first.wpilibj2.command.Command;
import java.util.function.DoubleSupplier;

public class Flywheel extends TalonFXSubsystem {
  public static DoubleSupplier shootVel;

  private VelocityTorqueCurrentFOC velocityTorqueCurrentFOC = new VelocityTorqueCurrentFOC(0.0);

  public Flywheel(TalonFXSubsystemConfiguration config) {
    super(config);
  }

  @Override
  public void periodic() {
    super.periodic();
  }

  public boolean atVel() {
    return Math.abs(getError()) <= 5.0;
  }

  public void setTargetVelocityTorqueCurrent(double velocity, double torqueFeedForward) {
    super.setpoint = velocity;
    super.inVelocityMode = true;
    setMaster(velocityTorqueCurrentFOC.withVelocity(velocity).withFeedForward(torqueFeedForward));
    if (shouldSimulate()) {
      simulationThread.setSimVoltage(
          () -> simController.calculate(getVelocity(), velocity) + torqueFeedForward);
    }
  }

  public Command setTargetVelocityTorqueCurrentCommand(double velocity, double torqueFeedForward) {
    return run(
        () -> {
          setTargetVelocityTorqueCurrentCommand(velocity, torqueFeedForward);
        });
  }
}
