package org.firstinspires.ftc.teamcode.utils;

/**
 * Custom Button that takes a BooleanSupplier for flexible trigger conditions.
 * Allows combining multiple gamepad inputs into a single button.
 */

import com.arcrobotics.ftclib.command.button.Button;

import java.util.function.BooleanSupplier;

public class FunctionalButton extends Button {
    private final BooleanSupplier booleanSupplier;

    public FunctionalButton(BooleanSupplier supplier) {
        booleanSupplier = supplier;
    }

    @Override
    public boolean get() {
        return booleanSupplier.getAsBoolean();
    }
}

// Special thanks to PeterLu for contributions to this code. All code and interpretation rights belong to PeterLu.
