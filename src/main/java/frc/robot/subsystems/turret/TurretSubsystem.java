package frc.robot.subsystems;

import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.hardware.TalonFX;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class TurretSubsystem extends SubsystemBase {

  private final TalonFX turretMotor = new TalonFX(3); // change ID

  private final NetworkTable limelightTable;

  // Tunables
  private static final double kP = 0.035; // start small, tune up
  private static final double kMaxOutput = 4.0; // volts
  private static final double kDeadband = 0.5; // degrees

  private final VoltageOut voltageRequest = new VoltageOut(0);

  public TurretSubsystem() {
    limelightTable = NetworkTableInstance.getDefault().getTable("limelight-three");

    turretMotor.setPosition(0);
  }

  @Override
  public void periodic() {
    trackTarget();
    //    turretMotor.set(0.2);
  }

  private void trackTarget() {
    double tx = limelightTable.getEntry("tx").getDouble(0.0);
    double tv = limelightTable.getEntry("tv").getDouble(0.0);

    // If no target, stop turret
    if (tv < 1.0) {
      turretMotor.setControl(voltageRequest.withOutput(0));
      return;
    }

    // Deadband
    if (Math.abs(tx) < kDeadband) {
      turretMotor.setControl(voltageRequest.withOutput(0));
      return;
    }

    // Proportional control
    double output = tx * kP;

    // Clamp voltage
    output = MathUtil.clamp(output, -kMaxOutput, kMaxOutput);

    turretMotor.setControl(voltageRequest.withOutput(output));
  }
}
