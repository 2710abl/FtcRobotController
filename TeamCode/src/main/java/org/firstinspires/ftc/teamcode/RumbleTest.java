package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;

public class RumbleTest extends OpMode {
    double endGameStart;
    boolean isEndGame;

    @Override
    public void init() {

    }

    @Override
    public void start() {
        endGameStart = getRuntime() + 100;
    }

    @Override
    public void loop() {
        // end game check
        if (endGameStart >= getRuntime() && !isEndGame) {
            gamepad1.rumbleBlips(3);
            gamepad2.rumbleBlips(3);
            telemetry.addLine("End Game: 20s Remaining! Return to base.");
            telemetry.update();
            isEndGame = true;
        }
    }
}
