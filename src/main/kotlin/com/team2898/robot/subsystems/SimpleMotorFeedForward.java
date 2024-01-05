package com.team2898.robot.subsystems;

import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.Nat;
import edu.wpi.first.math.system.plant.LinearSystemId;
public class SimpleMotorFeedForward {
    public final double ks;
    public final double kv;
    public final double ka;

    public SimpleMotorFeedForward(double ks, double kv, double ka) {
        this.ks = ks;
        this.kv = kv;
        this.ka = ka;
    }

    public SimpleMotorFeedForward(double ks, double kv) {
        this(ks, kv, 0);
    }

    public double calculate(double velocity, double acceleration) {
        return ks * Math.signum(velocity) + kv * velocity + ka * acceleration;
    }

    public double calculate(double currentVelocity, double nextVelocity, double dtSeconds) {
        var plant = LinearSystemId.identifyVelocitySystem(this.kv, this.ka);
        var feedforward = new LinearPlantInversionFeedforward<>(plant, dtSeconds);

        var r = Matrix.mat(Nat.N1(), Nat.N1()).fill(currentVelocity);
        var nextR = Matrix.mat(Nat.N1(), Nat.N1()).fill(nextVelocity);

        return ks * Math.signum(currentVelocity) + feedforward.calculate(r, nextR).get(0, 0);
    }

    public double calculate(double velocity) {
        return calculate(velocity, 0);
    }
    public double maxAchievableVelocity(double maxVoltage, double acceleration) {
        // Assume max velocity is positive
        return (maxVoltage - ks - acceleration * ka) / kv;
    }
    public double minAchievableVelocity(double maxVoltage, double acceleration) {
        // Assume min velocity is negative, ks flips sign
        return (-maxVoltage + ks - acceleration * ka) / kv;
    }
    public double maxAchievableAcceleration(double maxVoltage, double velocity) {
        return (maxVoltage - ks * Math.signum(velocity) - velocity * kv) / ka;
    }

    public double minAchievableAcceleration(double maxVoltage, double velocity) {
        return maxAchievableAcceleration(-maxVoltage, velocity);
    }
}
