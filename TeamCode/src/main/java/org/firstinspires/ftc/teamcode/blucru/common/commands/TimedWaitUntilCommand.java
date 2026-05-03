package org.firstinspires.ftc.teamcode.blucru.common.commands;

import com.seattlesolvers.solverslib.command.CommandBase;

import java.util.function.BooleanSupplier;

public class TimedWaitUntilCommand extends CommandBase {
    double maxTime;
    double startTime;
    BooleanSupplier conditional;
    public TimedWaitUntilCommand(double time, BooleanSupplier supplier){
        maxTime = time;
        conditional = supplier;
    }

    @Override
    public void initialize() {
        super.initialize();
        startTime = System.currentTimeMillis();
    }

    @Override
    public boolean isFinished(){
        return conditional.getAsBoolean() || maxTime < System.currentTimeMillis() - startTime;
    }
}
