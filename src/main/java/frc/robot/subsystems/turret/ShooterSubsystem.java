package frc.robot.subsystems.turret;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.NeutralModeValue;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.interpolation.InterpolatingDoubleTreeMap;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class ShooterSubsystem extends SubsystemBase {
  private static final ShooterSubsystem INSTANCE = new ShooterSubsystem();

  public static ShooterSubsystem getInstance() {
    return INSTANCE;
  }

  public enum ShooterState {
    IDLE,
    SPIN_UP,
    RAPID_FIRE,
    RAPID_FIRE_ACCURATE,
    EMPTY
  }

  private ShooterState state = ShooterState.IDLE;

  static final double kS = 0.2; // static friction (volts)
  static final double kV = 0.12; // volts per RPS
  static final double kA = 0.01; // volts per RPS^2
  static final double kP = 1.0;
  static final double kI = 0.0;
  static final double kD = 0.05;

  private final TalonFX shooterMotor = new TalonFX(1); // chjange ID
  private static final double GEAR_RATIO = 1.0; // 1:1

  private double lastRPM = 0.0;

  SimpleMotorFeedforward feedforward = new SimpleMotorFeedforward(kS, kV, kA);
  PIDController velocityPID = new PIDController(kP, kI, kD);

  // config
  private final double overspinFactor = 1.1;
  private final double rpmRapidFireTolerance = 200;
  private final double rpmAccurateTolerance = 50;
  boolean shotRecentlyFired = false;
  private double lastTargetRPS = 0.0;
  private final VoltageOut voltageRequest = new VoltageOut(0);

  private static final InterpolatingDoubleTreeMap shotFlywheelSpeedMap =
      new InterpolatingDoubleTreeMap();

  public ShooterSubsystem() {
    var config = new TalonFXConfiguration();
    config.MotorOutput.NeutralMode = NeutralModeValue.Coast;
    config.CurrentLimits.SupplyCurrentLimit = 40.0;
    config.CurrentLimits.SupplyCurrentLimitEnable = true;
    shooterMotor.getConfigurator().apply(config);

    // this is example later replace
    shotFlywheelSpeedMap.put(2.0, 4200.0); // its distance and second one is rpm
  }

  @Override
  public void periodic() {
    double distance = getTargetDistance(); // change

    double baseRPM = shotFlywheelSpeedMap.get(distance);
    double targetRPM = baseRPM * overspinFactor;
    double currentRPM = getVelocityRevPerSec() * 60.0;

    double targetRPS = targetRPM / 60.0;
    double currentRPS = currentRPM / 60.0;

    double rpmDrop = lastRPM - currentRPM;
    shotRecentlyFired = rpmDrop > 300; // tune
    lastRPM = currentRPM;
    switch (state) {
      case IDLE:
      case SPIN_UP:
        double voltage = setShooterRPM(targetRPS, currentRPS);
        shooterMotor.setControl(voltageRequest.withOutput(voltage));
        break;
      case RAPID_FIRE:
        if (Math.abs(currentRPM - targetRPM) <= rpmRapidFireTolerance) {
          // feed balls
        } else {
          // dont feed
        }
        break;
      case RAPID_FIRE_ACCURATE:
        if (Math.abs(currentRPM - targetRPM) <= rpmAccurateTolerance) {
          // feed
        } else {
          // no no feed
        }
        break;
      case EMPTY:
        shooterMotor.setControl(voltageRequest.withOutput(0.0));
        break;
    }
  }

  double getVelocityRevPerSec() {
    double motorRPS = shooterMotor.getVelocity().getValueAsDouble();
    return (motorRPS / GEAR_RATIO);
  }

  public double setShooterRPM(double targetRPS, double currentRPS) {
    //    double accelleration = (targetRPS - lastTargetRPS) / 0.02;
    double ffVolts = feedforward.calculate(targetRPS);

    double pidVolts = velocityPID.calculate(currentRPS, targetRPS);
    double output = ffVolts + pidVolts;
    if (shotRecentlyFired) {
      double rpmError = (targetRPS - currentRPS) * 60.0;
      double boost = Math.min(rpmError / 100.0, 4.0);
      output += boost;
    }
    lastTargetRPS = targetRPS;
    output = MathUtil.clamp(output, -12.0, 12.0);
    return output;
  }

  public void setState(ShooterState newState) {
    state = newState;
  }

  private double getTargetDistance() {
    // find distance somehow idk prob pose
    return 0.0;
  }
}
