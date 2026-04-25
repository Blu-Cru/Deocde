package org.firstinspires.ftc.teamcode.blucru.opmodes.auto;

import android.icu.text.SelectFormat;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.sfdev.assembly.state.StateMachine;
import com.sfdev.assembly.state.StateMachineBuilder;

import org.firstinspires.ftc.teamcode.blucru.common.util.Alliance;
import org.firstinspires.ftc.teamcode.blucru.opmodes.BluLinearOpMode;


@Autonomous(name = "Auto", group = "1")
public class Auto extends BluLinearOpMode {
    public boolean selectedauto = false;
    public BaseAuto autoToRun;

    enum State {
        ALLIANCE_PICK,
        AUTO_PICK,
        INITAlIZE,
        INITIALIZED,
        RUNNING,
        RESETTING
    }

    enum AUTOSTARTINGPOS {
        CLOSE_AUTO,
        FAR_BLUE_AUTO,
        FAR_BLUE_AUTO_SWEEP
    }
    Alliance CurrentSelectedAlliance = Alliance.BLUE;
    AUTOSTARTINGPOS CurrentSelectedAuto = AUTOSTARTINGPOS.CLOSE_AUTO;
    StateMachine sm;

    @Override
    public void initialize() {
        robot.clear();
        robot.addTurretCam();
        addSixWheel();
        addIntake();
        addElevator();
        addShooter();
        addTurret();
        addTransfer();
        addLLTagDetector();

        sm = new StateMachineBuilder()
                .state(State.ALLIANCE_PICK)
                .loop(() -> {
                    telemetry.addLine("Press Right Bumper to Confirm Selection! >.<");
                    if(CurrentSelectedAlliance == Alliance.BLUE) {
                        telemetry.addLine("BLUE <--");
                        telemetry.addLine("RED");
                    } else if (CurrentSelectedAlliance == Alliance.RED) {
                        telemetry.addLine("BLUE");
                        telemetry.addLine("RED <--");
                    }
                    if(driver1.pressedDpadDown()) {
                        if(CurrentSelectedAlliance == Alliance.BLUE) CurrentSelectedAlliance = Alliance.RED;
                    } else if (driver1.pressedDpadUp()) {
                        if (CurrentSelectedAlliance == Alliance.RED) CurrentSelectedAlliance = Alliance.BLUE;
                    }
                    //telemetry.update();
                })
                .transition(() -> driver1.pressedRightBumper(), State.AUTO_PICK)

                .state(State.AUTO_PICK)
                .loop(() -> {
                    telemetry.addData("Alliance", CurrentSelectedAlliance);
                    telemetry.addLine("Press Right Bumper to Confirm Selection! >.<");

                    for (AUTOSTARTINGPOS autoOption : AUTOSTARTINGPOS.values()) {
                        telemetry.addLine(getAutoDisplayName(autoOption)
                                + (autoOption == CurrentSelectedAuto ? " <--" : ""));
                    }

                    if (driver1.pressedDpadDown()) {
                        CurrentSelectedAuto = cycleAutoSelection(1);
                    } else if (driver1.pressedDpadUp()) {
                        CurrentSelectedAuto = cycleAutoSelection(-1);
                    }
                    //telemetry.update();
                })
                .transition(() -> driver1.pressedRightBumper(), State.INITAlIZE)

                .state(State.INITAlIZE)
                .onEnter(() -> {
                    selectedauto = true;
                    telemetry.addLine("Building Paths . . .");
                    telemetry.update();

                    // Map selection to AutoConfig Enum
                    AutoConfig.AUTOS autoEnum = null;
                    if (CurrentSelectedAuto == AUTOSTARTINGPOS.CLOSE_AUTO) autoEnum = AutoConfig.AUTOS.CLOSE_AUTO;
                    else if (CurrentSelectedAuto == AUTOSTARTINGPOS.FAR_BLUE_AUTO) autoEnum = AutoConfig.AUTOS.FAR_BLUE_AUTO;
                    else if (CurrentSelectedAuto == AUTOSTARTINGPOS.FAR_BLUE_AUTO_SWEEP) autoEnum = AutoConfig.AUTOS.FAR_BLUE_AUTO_SWEEP;

                    // Instantiate selected auto
                    autoToRun = AutoConfig.getAutoInstance(autoEnum);

                    if (autoToRun != null) {
                        // Inject hardware references
                        autoToRun.hardwareMap = Auto.this.hardwareMap;
                        autoToRun.telemetry = Auto.this.telemetry;
                        autoToRun.gamepad1 = Auto.this.gamepad1;
                        autoToRun.gamepad2 = Auto.this.gamepad2;
                        autoToRun.robot = Auto.this.robot;
                        autoToRun.sixWheel = Auto.this.sixWheel;
                        autoToRun.intake = Auto.this.intake;
                        autoToRun.elevator = Auto.this.elevator;
                        autoToRun.shooter = Auto.this.shooter;
                        autoToRun.turret = Auto.this.turret;
                        autoToRun.transfer = Auto.this.transfer;
                        autoToRun.obeliskTagDetector = Auto.this.obeliskTagDetector;
                        autoToRun.llTagDetector = Auto.this.llTagDetector;
                        autoToRun.driver1 = Auto.this.driver1;
                        autoToRun.driver2 = Auto.this.driver2;

                        autoToRun.initialize();
                    }
                })
                .transition(() -> !AutoConfig.InitBusy(), State.INITIALIZED)

                .state(State.INITIALIZED)
                .loop(() -> {
                    telemetry.addLine("Paths Built!");
                    telemetry.addLine("Initalized!");
                    telemetry.addLine("Congrats, do a dance!");
                    //telemetry.update();
                })
                .build();

        sm.setState(State.ALLIANCE_PICK);
        sm.start();
    }
    
    @Override
    public void initializePeriodic() {
        if (sm != null) sm.update();
        if (autoToRun != null) {
            autoToRun.initializePeriodic();
        }
    }

    public void onStart() {
        if(selectedauto) {
            if (autoToRun != null) {
                autoToRun.onStart();
            }
            sm.stop();
        } else {
            throw new RuntimeException("Auto not selected! You silly billy!");
        }
    }

    public void periodic() {
        if (autoToRun != null) {
            try {
                autoToRun.periodic();
            } catch (Exception e) {
                telemetry.addData("Error", e.getMessage());
            }
        }
    }

    private AUTOSTARTINGPOS cycleAutoSelection(int delta) {
        AUTOSTARTINGPOS[] values = AUTOSTARTINGPOS.values();
        int nextIndex = (CurrentSelectedAuto.ordinal() + delta + values.length) % values.length;
        return values[nextIndex];
    }

    private String getAutoDisplayName(AUTOSTARTINGPOS autoOption) {
        switch (autoOption) {
            case CLOSE_AUTO:
                return "closeauto";
            case FAR_BLUE_AUTO:
                return "farblueauto";
            case FAR_BLUE_AUTO_SWEEP:
                return "farblueautosweep";
            default:
                return autoOption.name().toLowerCase();
        }
    }
}
