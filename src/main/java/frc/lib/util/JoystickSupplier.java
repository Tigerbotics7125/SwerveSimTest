/**
 * Copyright (C) 2022, Tigerbotics' team members and all other contributors.
 * Open source software; you can modify and/or share this software.
 */
package frc.lib.util;

import static tigerlib.Util.*;

import java.util.function.Supplier;

public interface JoystickSupplier extends Supplier<Double> {

    default Double getClean() {
        double value = get();

        value = deadband(value, .075);
        value = clamp(value, -1, 1);

        return value;
    }
}
