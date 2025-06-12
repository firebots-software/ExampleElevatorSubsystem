package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.PositionOut;
import com.ctre.phoenix6.signals.NeutralModeValue;
import com.ctre.phoenix6.signals.InvertedValue;

public class ElevatorSubsystem extends SubsystemBase {
    // Rotations per meter refers to how many full rotations of the motor correlate to one meter up by the elevator
    private static final double kRotationsPerMeter = 10.0;

    // TalonFX is the name of the motor controllers we use (essentially the motor in code)
    private final TalonFX motor;  
    private double targetHeight;

    public ElevatorSubsystem() {
        // The number within the parentheses refers to the ID we assign to the motor (so we can differentiate between motors)
        motor = new TalonFX(1);
    }

    public void setHeight(double heightMeters) {
        targetHeight = heightMeters;
        motor.setControl(new PositionOut(heightMeters * kRotationsPerMeter));
    }

    public double getHeight() {
        return motor.getRotorPosition().getValue() / kRotationsPerMeter;
    }

    // Sets the encoder to read 0
    public void zeroEncoder() {
        motor.getRotorPosition().set(0.0);
    }

    public boolean isAtTargetHeight(double toleranceMeters) {
        return Math.abs(getHeight() - targetHeight) <= toleranceMeters;
    }

    public void stop() {
        setHeight(getHeight());
    }
}
