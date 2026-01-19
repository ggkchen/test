package frc.robot.subsystems.turret;

import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.hardware.TalonFX;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj2.command.SubsystemBase;


public class TurretSubsystem extends SubsystemBase {


  //Constants
  private final double kmaxAccelleration = Math.toRadians(999999999);  //    4 rad/s^2 max
  //TODO: tune
  private final double kmaxVelocity = Math.toRadians(10);    //10 rad/s max
  private static final double kP = 0.035;   //Proportional
  private static final double kI = 0.001;   //Integral
  private static final double kD = 0.001;   //Derivative
  private static final double kMaxOutput = 4.0; // volts
  private static final double kDeadband = 0.5; // degrees


  private final TalonFX turretMotor = new TalonFX(3); // change ID
  private final NetworkTable limelightTable;

  //
  private TrapezoidProfile profile =
          new TrapezoidProfile(
                  new TrapezoidProfile.Constraints(kmaxVelocity, kmaxVelocity));
private TrapezoidProfile.State setpoint = new TrapezoidProfile.State();




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


    double goalAngle = turretMotor.getPosition().getValueAsDouble() + tx;
    TrapezoidProfile.State goalState = new TrapezoidProfile.State(goalAngle, 0.0);

    setpoint = profile.calculate(
            0.02,
            setpoint,
            goalState);
    goalAngle = MathUtil.clamp(goalAngle, -180, 180);

    double posError = setpoint.position * turretMotor.getPosition().getValueAsDouble(); //TODO: add the minus current position
    double velError = setpoint.velocity * turretMotor.getVelocity().getValueAsDouble(); //TODO: add the minus current velocity
    double outputVolts = kP * posError
                      + kD * velError;

//    // Clamp voltage
//    output = MathUtil.clamp(output, -kMaxOutput, kMaxOutput);

    turretMotor.setVoltage(outputVolts);
//    turretMotor.getPosition()
  }
}
