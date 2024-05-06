package com.NewtonSwerve.Mk4;

import java.util.Objects;

import com.NewtonSwerve.ModuleConfig;

/**
 * Additional Mk4 module configuration parameters.
 * <p>
 * The configuration parameters here are used to customize the behavior of the
 * Mk4 swerve module.
 * Each setting is initialized to a default that should be adequate for most use
 * cases.
 */
public class Mk4ModuleConfiguration extends ModuleConfig {
    @Override
    public boolean equals(Object o) {
        if (this == o)
            return true;
        if (o == null || getClass() != o.getClass())
            return false;
        Mk4ModuleConfiguration that = (Mk4ModuleConfiguration) o;
        return Double.compare(that.getNominalVoltage(), getNominalVoltage()) == 0;
    }

    @Override
    public int hashCode() {
        return Objects.hash(getNominalVoltage());
    }

    @Override
    public String toString() {
        return "Mk4ModuleConfiguration{" +
                "nominalVoltage=" + nominalVoltage +
                '}';
    }
}
