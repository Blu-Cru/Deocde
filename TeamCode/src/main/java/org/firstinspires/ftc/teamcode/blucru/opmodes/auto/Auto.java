package org.firstinspires.ftc.teamcode.blucru.opmodes.auto;

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
        FAR,
        CLOSE,
        CLOSE_MOTIF,
        ROOT_NEGATIVE_ONE
    }
    Alliance CurrentSelectedAlliance = Alliance.BLUE;
    AUTOSTARTINGPOS CurrentSelectedAuto = AUTOSTARTINGPOS.CLOSE;
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
                        if(CurrentSelectedAlliance == Alliance.RED) CurrentSelectedAlliance = Alliance.BLUE;
                    } else if (driver1.pressedDpadUp()) {
                        if (CurrentSelectedAlliance == Alliance.RED)
                            CurrentSelectedAlliance = Alliance.BLUE;
                        if(CurrentSelectedAlliance == Alliance.BLUE) CurrentSelectedAlliance = Alliance.RED;
                    }
                    //telemetry.update();
                })
                .transition(() -> driver1.pressedRightBumper(), State.AUTO_PICK)

                .state(State.AUTO_PICK)
                .loop(() -> {
                    telemetry.addData("Alliance", CurrentSelectedAlliance);
                    telemetry.addLine("Press Right Bumper to Confirm Selection! >.<");
                    
                    if(CurrentSelectedAlliance == Alliance.BLUE) {
                        if(CurrentSelectedAuto == AUTOSTARTINGPOS.CLOSE) {
                            telemetry.addLine("Blue Close <--");
                            telemetry.addLine("Blue Far");
                            telemetry.addLine("Blue Close Motif");
                            telemetry.addLine("Blue Root Negative One");
                        } else if (CurrentSelectedAuto == AUTOSTARTINGPOS.FAR) {
                            telemetry.addLine("Blue Close");
                            telemetry.addLine("Blue Far <--");
                            telemetry.addLine("Blue Close Motif");
                            telemetry.addLine("Blue Root Negative One");
                        } else if (CurrentSelectedAuto == AUTOSTARTINGPOS.CLOSE_MOTIF) {
                            telemetry.addLine("Blue Close");
                            telemetry.addLine("Blue Far");
                            telemetry.addLine("Blue Close Motif <--");
                            telemetry.addLine("Blue Root Negative One");
                        } else if (CurrentSelectedAuto == AUTOSTARTINGPOS.ROOT_NEGATIVE_ONE) {
                            telemetry.addLine("Blue Close");
                            telemetry.addLine("Blue Far");
                            telemetry.addLine("Blue Close Motif");
                            telemetry.addLine("Blue Root Negative One <--");
                        }
                    } else if (CurrentSelectedAlliance == Alliance.RED) {
                        if(CurrentSelectedAuto == AUTOSTARTINGPOS.CLOSE) {
                            telemetry.addLine("Red Close <--");
                            telemetry.addLine("Red Far");
                            telemetry.addLine("Red Close Motif");
                        } else if (CurrentSelectedAuto == AUTOSTARTINGPOS.FAR) {
                            telemetry.addLine("Red Close");
                            telemetry.addLine("Red Far <--");
                            telemetry.addLine("Red Close Motif");
                        } else if (CurrentSelectedAuto == AUTOSTARTINGPOS.CLOSE_MOTIF) {
                            telemetry.addLine("Red Close");
                            telemetry.addLine("Red Far");
                            telemetry.addLine("Red Close Motif <--");
                        }
                    }
                    
                    if(driver1.pressedDpadDown()) {
                        if(CurrentSelectedAuto == AUTOSTARTINGPOS.CLOSE) CurrentSelectedAuto = AUTOSTARTINGPOS.FAR;
                        else if(CurrentSelectedAuto == AUTOSTARTINGPOS.FAR) CurrentSelectedAuto = AUTOSTARTINGPOS.CLOSE_MOTIF;
                        else if(CurrentSelectedAuto == AUTOSTARTINGPOS.CLOSE_MOTIF) CurrentSelectedAuto = AUTOSTARTINGPOS.ROOT_NEGATIVE_ONE;
                        else if(CurrentSelectedAuto == AUTOSTARTINGPOS.ROOT_NEGATIVE_ONE) CurrentSelectedAuto = AUTOSTARTINGPOS.CLOSE;
                    } else if (driver1.pressedDpadUp()) {
                        if(CurrentSelectedAuto == AUTOSTARTINGPOS.CLOSE) CurrentSelectedAuto = AUTOSTARTINGPOS.ROOT_NEGATIVE_ONE;
                        else if(CurrentSelectedAuto == AUTOSTARTINGPOS.ROOT_NEGATIVE_ONE) CurrentSelectedAuto = AUTOSTARTINGPOS.CLOSE_MOTIF;
                        else if(CurrentSelectedAuto == AUTOSTARTINGPOS.CLOSE_MOTIF) CurrentSelectedAuto = AUTOSTARTINGPOS.FAR;
                        else if(CurrentSelectedAuto == AUTOSTARTINGPOS.FAR) CurrentSelectedAuto = AUTOSTARTINGPOS.CLOSE;
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
                    if (CurrentSelectedAlliance == Alliance.BLUE) {
                        if (CurrentSelectedAuto == AUTOSTARTINGPOS.CLOSE) autoEnum = AutoConfig.AUTOS.BLUE_CLOSE;
                        else if (CurrentSelectedAuto == AUTOSTARTINGPOS.FAR) autoEnum = AutoConfig.AUTOS.BLUE_FAR;
                        else if (CurrentSelectedAuto == AUTOSTARTINGPOS.CLOSE_MOTIF) autoEnum = AutoConfig.AUTOS.BLUE_CLOSE_MOTIF;
                        else if (CurrentSelectedAuto == AUTOSTARTINGPOS.ROOT_NEGATIVE_ONE) autoEnum = AutoConfig.AUTOS.ROOT_NEGATIVE_ONE;
                    } else {
                        if (CurrentSelectedAuto == AUTOSTARTINGPOS.CLOSE) autoEnum = AutoConfig.AUTOS.RED_CLOSE;
                        else if (CurrentSelectedAuto == AUTOSTARTINGPOS.FAR) autoEnum = AutoConfig.AUTOS.RED_FAR;
                        else if (CurrentSelectedAuto == AUTOSTARTINGPOS.CLOSE_MOTIF) autoEnum = AutoConfig.AUTOS.RED_CLOSE_MOTIF;
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
}
