package org.firstinspires.ftc.teamcode.blucru.opmodes.auto;

import android.icu.text.SelectFormat;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.sfdev.assembly.state.StateMachine;
import com.sfdev.assembly.state.StateMachineBuilder;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.blucru.common.subsytems.Robot;
import org.firstinspires.ftc.teamcode.blucru.common.util.Alliance;
import org.firstinspires.ftc.teamcode.blucru.common.util.Globals;
import org.firstinspires.ftc.teamcode.blucru.common.util.Pose2d;
import com.qualcomm.robotcore.util.ElapsedTime;
import org.firstinspires.ftc.teamcode.blucru.opmodes.BluLinearOpMode;

@Autonomous(name = "Auto", group = "norm", preselectTeleOp = "Tele")
public class Auto extends BluLinearOpMode {
    public boolean selectedauto = false;
    public BaseAuto autoToRun;
    public ElapsedTime errorTimer = new ElapsedTime();
    public boolean showError = false;

    enum State {
        ALLIANCE_PICK,
        AUTO_PICK,
        INITAlIZE,
        INITIALIZED,
        RUNNING,
        RESETTING
    }

    enum AUTOSTARTINGPOS {
        CLOSE,
        CLOSE_MOTIF,
        FAR,
        FAR_SWEEP
    }
    Alliance CurrentSelectedAlliance = Alliance.BLUE;
    AUTOSTARTINGPOS CurrentSelectedAuto = AUTOSTARTINGPOS.CLOSE;
    StateMachine sm;

    @Override
    public void initialize() {
        manageTelemetry = true;
        robot.clear();

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
                .transition(() -> driver1.pressedRightBumper(), State.AUTO_PICK, () -> {
                    Globals.setAlliance(CurrentSelectedAlliance);
                })

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
                    Globals.setAlliance(CurrentSelectedAlliance);
                    telemetry.addLine("Building Paths . . .");
                    telemetry.update();

                    // Map selection to AutoConfig Enum
                    AutoConfig.AUTOS autoEnum = null;
                    if (CurrentSelectedAlliance == Alliance.BLUE) {
                        if (CurrentSelectedAuto == AUTOSTARTINGPOS.CLOSE) autoEnum = AutoConfig.AUTOS.CLOSE_BLUE;
                        else if (CurrentSelectedAuto == AUTOSTARTINGPOS.FAR) autoEnum = AutoConfig.AUTOS.FAR_BLUE;
                        else if (CurrentSelectedAuto == AUTOSTARTINGPOS.FAR_SWEEP) autoEnum = AutoConfig.AUTOS.FAR_BLUE_SWEEP;
                        else if (CurrentSelectedAuto == AUTOSTARTINGPOS.CLOSE_MOTIF) autoEnum = AutoConfig.AUTOS.CLOSE_BLUE_MOTIF;
                    } else {
                        // RED ALLIANCE
                        if (CurrentSelectedAuto == AUTOSTARTINGPOS.CLOSE) autoEnum = AutoConfig.AUTOS.CLOSE_RED;
                        else if (CurrentSelectedAuto == AUTOSTARTINGPOS.FAR) autoEnum = AutoConfig.AUTOS.FAR_RED;
                        else if (CurrentSelectedAuto == AUTOSTARTINGPOS.FAR_SWEEP) autoEnum = AutoConfig.AUTOS.FAR_RED_SWEEP;
                        else if (CurrentSelectedAuto == AUTOSTARTINGPOS.CLOSE_MOTIF) autoEnum = AutoConfig.AUTOS.CLOSE_RED_MOTIF;
                    }

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
                        robot.init();
                    }
                })
                .transition(() -> !AutoConfig.InitBusy(), State.INITIALIZED)

                .state(State.INITIALIZED)
                .loop(() -> {
                    telemetry.addLine("Paths Built!");
                    if (showError && errorTimer.milliseconds() < 2000) {
                        telemetry.addLine("Red Sweep doesn't exist yet! Falling back to Far Red!");
                    }
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

    @Override
    public void telemetry() {
        if (!selectedauto || autoToRun == null) {
            return; // selection UI handles its own telemetry during init
        }

        // Read live subsystem refs from the singleton because the selected auto owns
        // the active subsystem set.
        Robot r = Robot.getInstance();

        telemetry.addLine("======== AUTO ========");
        telemetry.addData("Selected", autoToRun.getClass().getSimpleName());
        telemetry.addData("Alliance", Globals.alliance);
        telemetry.addData("Match Time", "%.2f s", Globals.matchTime.seconds());
        if (autoToRun.sm != null) {
            telemetry.addData("State", autoToRun.sm.getState());
        }
        autoToRun.autoTelemetry(telemetry);

        telemetry.addLine();
        telemetry.addLine("======== ROBOT ========");
        if (r.sixWheelDrivetrain != null) {
            Pose2d pose = r.sixWheelDrivetrain.getPos();
            telemetry.addData("Pose", "(%.1f, %.1f, %.1f°)",
                    pose.getX(), pose.getY(), Math.toDegrees(pose.getH()));
        }
        telemetry.addData("Voltage", "%.2f V", Globals.voltage);
        telemetry.addData("Pose History", r.positionHistory.size());
        telemetry.addData("Subsystems", r.getAmountOfSubsystems());

        if (r.sixWheelDrivetrain != null) {
            section("DRIVETRAIN");
            r.sixWheelDrivetrain.telemetry(telemetry);
        }
        if (r.intake != null) {
            section("INTAKE");
            r.intake.telemetry(telemetry);
        }
        if (r.elevator != null) {
            section("ELEVATOR");
            r.elevator.telemetry(telemetry);
        }
        if (r.transfer != null) {
            section("TRANSFER");
            r.transfer.telemetry(telemetry);
        }
        if (r.shooter != null) {
            section("SHOOTER");
            r.shooter.telemetry(telemetry);
        }
        if (r.turret != null) {
            section("TURRET");
            r.turret.telemetry(telemetry);
        }
        if (r.turretCam != null) {
            section("TURRET CAM");
            r.turretCam.telemetry(telemetry);
        }
    }

    private void section(String name) {
        telemetry.addLine("");
        telemetry.addLine("---- " + name + " ----");
    }

    private AUTOSTARTINGPOS cycleAutoSelection(int delta) {
        AUTOSTARTINGPOS[] values = AUTOSTARTINGPOS.values();
        int nextIndex = (CurrentSelectedAuto.ordinal() + delta + values.length) % values.length;
        return values[nextIndex];
    }

    private String getAutoDisplayName(AUTOSTARTINGPOS autoOption) {
        switch (autoOption) {
            case CLOSE:
                return "Close Auto";
            case FAR:
                return "Far Auto";
            case FAR_SWEEP:
                return "Far Auto Sweep";
            case CLOSE_MOTIF:
                return "Close Motif Auto";
            default:
                return autoOption.name().toLowerCase();
        }
    }
}
