package org.firstinspires.ftc.teamcode.blucru.common.commands;

import java.util.function.BooleanSupplier;

public class GeneralBooleanSupplier implements BooleanSupplier {
    boolean ret;
    public GeneralBooleanSupplier(boolean bool){
        ret = bool;
    }
    @Override
    public boolean getAsBoolean() {
        return ret;
    }
}
