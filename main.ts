/*
 * CrabBot Mini - Standalone 8-DOF Quadruped Robot Extension for BBC micro:bit
 *
 * Hardware Layout (Servo Headers S1..S8 on PCA9685 @ 0x40):
 * - FRONT RIGHT (FR): Base = S1, Lift = S2
 * - REAR RIGHT  (RR): Base = S3, Lift = S4
 * - FRONT LEFT  (FL): Base = S5, Lift = S6
 * - REAR LEFT   (RL): Base = S7, Lift = S8
 */

enum CrabBotMode {
    //% block="stand"
    Stand = 0,
    //% block="walk forward"
    WalkForward = 1,
    //% block="walk backward"
    WalkBackward = 2,
    //% block="turn left"
    TurnLeft = 3,
    //% block="turn right"
    TurnRight = 4,
    //% block="spin left"
    SpinLeft = 5,
    //% block="spin right"
    SpinRight = 6,
    //% block="wave"
    Wave = 7,
    //% block="dance"
    Dance = 8,
    //% block="self balance"
    SelfBalance = 9,
    //% block="stop"
    Stop = 10
}

enum CrabBotTurn {
    //% block="left"
    Left = 1,
    //% block="right"
    Right = -1
}

//% color="#00A4A6" weight=28 icon="\uf188"
namespace CrabBotMini {
    // --- 0. Built-in PCA9685 I2C Servo Driver (0x40) ---
    const PCA9685_ADD = 0x40;
    const MODE1 = 0x00;
    const LED0_ON_L = 0x06;
    const PRESCALE = 0xFE;

    const S1 = 0;
    const S2 = 1;
    const S3 = 2;
    const S4 = 3;
    const S5 = 4;
    const S6 = 5;
    const S7 = 6;
    const S8 = 7;

    let pcaInitialized = false;

    function i2cWrite(addr: number, reg: number, value: number): void {
        const buf = pins.createBuffer(2);
        buf[0] = reg;
        buf[1] = value;
        pins.i2cWriteBuffer(addr, buf);
    }

    function i2cRead(addr: number, reg: number): number {
        pins.i2cWriteNumber(addr, reg, NumberFormat.UInt8BE);
        return pins.i2cReadNumber(addr, NumberFormat.UInt8BE);
    }

    function setFreq(freq: number): void {
        let prescaleval = 25000000;
        prescaleval /= 4096;
        prescaleval /= freq;
        prescaleval -= 1;
        const prescale = prescaleval;
        const oldmode = i2cRead(PCA9685_ADD, MODE1);
        const newmode = (oldmode & 0x7F) | 0x10;
        i2cWrite(PCA9685_ADD, MODE1, newmode);
        i2cWrite(PCA9685_ADD, PRESCALE, prescale);
        i2cWrite(PCA9685_ADD, MODE1, oldmode);
        control.waitMicros(5000);
        i2cWrite(PCA9685_ADD, MODE1, oldmode | 0xa1);
    }

    function initPCA9685(): void {
        i2cWrite(PCA9685_ADD, MODE1, 0x00);
        setFreq(50);
        pcaInitialized = true;
    }

    function setPwm(channel: number, on: number, off: number): void {
        if (channel < 0 || channel > 15) return;
        if (!pcaInitialized) {
            initPCA9685();
        }
        const buf = pins.createBuffer(5);
        buf[0] = LED0_ON_L + 4 * channel;
        buf[1] = on & 0xff;
        buf[2] = (on >> 8) & 0xff;
        buf[3] = off & 0xff;
        buf[4] = (off >> 8) & 0xff;
        pins.i2cWriteBuffer(PCA9685_ADD, buf);
    }

    function writeServo270(channel: number, value: number): void {
        const newvalue = Math.map(value, 0, 270, 0, 180);
        const us = newvalue * 1800 / 180 + 600;
        const pwm = us * 4096 / 20000;
        setPwm(channel, 0, pwm);
    }

    // --- 1. Leg Identifiers ---
    const LEG_FL = 0;
    const LEG_FR = 1;
    const LEG_RL = 2;
    const LEG_RR = 3;

    // --- 2. Calibration & Neutral X-Stance (30 deg Front / 30 deg Back) ---
    const BASE_FR_NEUTRAL = 200;
    const BASE_RR_NEUTRAL = 140;
    const BASE_FL_NEUTRAL = 35;
    const BASE_RL_NEUTRAL = 110;

    const DIR_FR = 1;
    const DIR_RR = 1;
    const DIR_FL = -1;
    const DIR_RL = -1;

    const LIFT_FR_REST = 85;
    const LIFT_RR_REST = 85;
    const LIFT_FL_REST = 260;
    const LIFT_RL_REST = 260;

    const LIFT_FR_DOWN = 85;
    const LIFT_RR_DOWN = 85;
    const LIFT_FL_DOWN = 260;
    const LIFT_RL_DOWN = 260;

    const LIFT_H_FR = 60;
    const LIFT_H_RR = 60;
    const LIFT_H_FL = -60;
    const LIFT_H_RL = -55;

    const SERVO_MIN: number[] = [165, 78, 112, 78, 10, 195, 82, 200];
    const SERVO_MAX: number[] = [235, 242, 172, 150, 65, 265, 138, 265];

    // --- 3. Stride & Differential Turning Parameters ---
    const SWING_AMP = 18;
    const MAX_SAFE_AMP = 25;
    const MAX_TURN_PCT = 70;
    const DEFAULT_TURN_PCT = 40;

    let ampFl = 18;
    let ampFr = 18;
    let ampRl = 18;
    let ampRr = 18;
    let gaitStepDelay = 24;
    let currentSpeed = 5;
    let currentTurnPct = DEFAULT_TURN_PCT;

    // --- 4. Happy Dance Calibration Parameters ---
    const DANCE_H_AMP = 22;
    const DANCE_V_AMP = 55;

    // --- 5. Self-Leveling Mode Parameters ---
    const LEVEL_DEADBAND_DEG = 2;
    const LEVEL_KP_100 = 40;
    const LEVEL_FILTER_ALPHA_100 = 60;
    const MAX_LEVEL_STEP = 5;
    const MAX_LEVEL_CROUCH = 55;
    const LEVEL_SETTLE_MS = 30;
    const LEVEL_PITCH_SIGN = 1;
    const LEVEL_ROLL_SIGN = 1;

    // --- 6. Precomputed Smooth Trajectory Tables (Index 0..6) ---
    const SWING_CUBIC_1000: number[] = [0, 74, 259, 500, 741, 926, 1000];
    const SWING_SIN_1000: number[] = [0, 500, 866, 1000, 866, 500, 0];

    // --- 7. Runtime & FSM State ---
    let isWalking = false;
    let isDancing = false;
    let isLeveling = false;
    let inGaitStance = false;

    let activeMode: CrabBotMode = CrabBotMode.Stand;
    let pendingMode: CrabBotMode = CrabBotMode.Stand;
    let hasPendingMode = false;
    let modeEndTimeMs = 0;

    let actionToken = 0;
    let activeToken = 0;
    let dispatcherStarted = false;

    // Self-Leveling runtime state
    let levelCalibrated = false;
    let levelPitchZero = 0;
    let levelRollZero = 0;
    let levelPitchFilt10 = 0;
    let levelRollFilt10 = 0;
    let levelPitchCmd = 0;
    let levelRollCmd = 0;
    let crouchFr = 0;
    let crouchRr = 0;
    let crouchFl = 0;
    let crouchRl = 0;

    // Last-Angle Cache (indices 0..7 for S1..S8)
    const lastServo: number[] = [
        BASE_FR_NEUTRAL, LIFT_FR_REST,
        BASE_RR_NEUTRAL, LIFT_RR_REST,
        BASE_FL_NEUTRAL, LIFT_FL_REST,
        BASE_RL_NEUTRAL, LIFT_RL_REST
    ];
    let servoInitialized = false;

    // --------------------------------------------------------------
    // Low-Level Validation, Anti-Jitter Servo & Trajectory Helpers
    // --------------------------------------------------------------

    function absVal(val: number): number {
        return val < 0 ? -val : val;
    }

    function clampVal(val: number, lo: number, hi: number): number {
        if (val < lo) return lo;
        if (val > hi) return hi;
        return val;
    }

    function clampServoAngle(idx: number, angle: number): number {
        return clampVal(angle, SERVO_MIN[idx], SERVO_MAX[idx]);
    }

    function isCancelled(): boolean {
        return actionToken != activeToken;
    }

    function driveServo(idx: number, port: number, angle: number, pauseMs: number): void {
        if (servoInitialized && isCancelled()) return;
        const clamped = clampServoAngle(idx, angle);
        const diff = absVal(clamped - lastServo[idx]);
        if (!servoInitialized || diff >= 2) {
            lastServo[idx] = clamped;
            writeServo270(port, clamped);
        }
        if (pauseMs > 0 && !(servoInitialized && isCancelled())) {
            basic.pause(pauseMs);
        }
    }

    function setServoExact(idx: number, port: number, angle: number, pauseMs: number): void {
        if (servoInitialized && isCancelled()) return;
        const clamped = clampServoAngle(idx, angle);
        if (!servoInitialized || lastServo[idx] != clamped) {
            lastServo[idx] = clamped;
            writeServo270(port, clamped);
        }
        if (pauseMs > 0 && !(servoInitialized && isCancelled())) {
            basic.pause(pauseMs);
        }
    }

    function calcSwingOffset(sIdx: number, amp: number): number {
        const c = SWING_CUBIC_1000[sIdx];
        return -amp + Math.idiv(2 * amp * c, 1000);
    }

    function calcStanceOffset(stanceStage: number, sIdx: number, amp: number): number {
        const step18 = stanceStage * 6 + sIdx;
        return amp - Math.idiv(2 * amp * step18, 18);
    }

    function calcSwingLift(z0: number, liftH: number, sIdx: number): number {
        return z0 + Math.idiv(liftH * SWING_SIN_1000[sIdx], 1000);
    }

    function interpCubic(startVal: number, endVal: number, sIdx: number): number {
        const c = SWING_CUBIC_1000[sIdx];
        return startVal + Math.idiv((endVal - startVal) * c, 1000);
    }

    function interpLinear(startVal: number, endVal: number, sIdx: number): number {
        return startVal + Math.idiv((endVal - startVal) * sIdx, 6);
    }

    function pauseInterruptible(totalMs: number): boolean {
        let rem = totalMs;
        while (rem > 0) {
            if (isCancelled()) return true;
            const step = rem < 20 ? rem : 20;
            basic.pause(step);
            rem -= step;
        }
        return isCancelled();
    }

    // --------------------------------------------------------------
    // Interruption-Safe Pose Recovery
    // --------------------------------------------------------------

    function recoverSingleLeg(
        hIdx: number,
        hPort: number,
        vIdx: number,
        vPort: number,
        baseNeutral: number,
        vDown: number,
        liftDir: number,
        recoveryToken: number
    ): boolean {
        const hStart = lastServo[hIdx];
        if (absVal(hStart - baseNeutral) < 4) {
            setServoExact(hIdx, hPort, baseNeutral, 2);
            return true;
        }
        for (let s = 1; s <= 6; s++) {
            if (actionToken != recoveryToken || isCancelled()) return false;
            const vAng = calcSwingLift(vDown, liftDir * 35, s);
            const hAng = interpCubic(hStart, baseNeutral, s);
            driveServo(vIdx, vPort, vAng, 4);
            driveServo(hIdx, hPort, hAng, 6);
        }
        setServoExact(vIdx, vPort, vDown, 2);
        setServoExact(hIdx, hPort, baseNeutral, 2);
        return actionToken == recoveryToken && !isCancelled();
    }

    function recoverToStance(): boolean {
        inGaitStance = false;
        const recoveryToken = activeToken;
        if (actionToken != recoveryToken) return false;

        if (
            absVal(lastServo[0] - BASE_FR_NEUTRAL) < 2 &&
            absVal(lastServo[1] - LIFT_FR_DOWN) < 2 &&
            absVal(lastServo[2] - BASE_RR_NEUTRAL) < 2 &&
            absVal(lastServo[3] - LIFT_RR_DOWN) < 2 &&
            absVal(lastServo[4] - BASE_FL_NEUTRAL) < 2 &&
            absVal(lastServo[5] - LIFT_FL_DOWN) < 2 &&
            absVal(lastServo[6] - BASE_RL_NEUTRAL) < 2 &&
            absVal(lastServo[7] - LIFT_RL_DOWN) < 2
        ) {
            return actionToken == recoveryToken;
        }

        if (
            absVal(lastServo[1] - LIFT_FR_DOWN) >= 2 ||
            absVal(lastServo[3] - LIFT_RR_DOWN) >= 2 ||
            absVal(lastServo[5] - LIFT_FL_DOWN) >= 2 ||
            absVal(lastServo[7] - LIFT_RL_DOWN) >= 2
        ) {
            const s0Start = lastServo[0];
            const s1Start = lastServo[1];
            const s3Start = lastServo[3];
            const s4Start = lastServo[4];
            const s5Start = lastServo[5];
            const s7Start = lastServo[7];
            const frInAir = absVal(s1Start - LIFT_FR_DOWN) > 12;
            const flInAir = absVal(s5Start - LIFT_FL_DOWN) > 12;
            for (let s = 1; s <= 6; s++) {
                if (actionToken != recoveryToken || isCancelled()) return false;
                if (frInAir) {
                    driveServo(0, S1, interpCubic(s0Start, BASE_FR_NEUTRAL, s), 3);
                }
                if (flInAir) {
                    driveServo(4, S5, interpCubic(s4Start, BASE_FL_NEUTRAL, s), 3);
                }
                driveServo(1, S2, interpCubic(s1Start, LIFT_FR_DOWN, s), 4);
                driveServo(5, S6, interpCubic(s5Start, LIFT_FL_DOWN, s), 4);
                driveServo(7, S8, interpCubic(s7Start, LIFT_RL_DOWN, s), 4);
                driveServo(3, S4, interpCubic(s3Start, LIFT_RR_DOWN, s), 4);
            }
        }

        if (actionToken != recoveryToken || isCancelled()) return false;

        setServoExact(1, S2, LIFT_FR_DOWN, 2);
        setServoExact(3, S4, LIFT_RR_DOWN, 2);
        setServoExact(5, S6, LIFT_FL_DOWN, 2);
        setServoExact(7, S8, LIFT_RL_DOWN, 2);

        if (!recoverSingleLeg(4, S5, 5, S6, BASE_FL_NEUTRAL, LIFT_FL_DOWN, -1, recoveryToken)) return false;
        if (!recoverSingleLeg(2, S3, 3, S4, BASE_RR_NEUTRAL, LIFT_RR_DOWN, 1, recoveryToken)) return false;
        if (!recoverSingleLeg(0, S1, 1, S2, BASE_FR_NEUTRAL, LIFT_FR_DOWN, 1, recoveryToken)) return false;
        if (!recoverSingleLeg(6, S7, 7, S8, BASE_RL_NEUTRAL, LIFT_RL_DOWN, -1, recoveryToken)) return false;

        return actionToken == recoveryToken && !isCancelled();
    }

    // --------------------------------------------------------------
    // Gait & Stance Implementation
    // --------------------------------------------------------------

    function setLegAmplitude(leg: number, amplitude: number): void {
        const clamped = clampVal(amplitude, -MAX_SAFE_AMP, MAX_SAFE_AMP);
        if (leg == LEG_FL) ampFl = clamped;
        else if (leg == LEG_FR) ampFr = clamped;
        else if (leg == LEG_RL) ampRl = clamped;
        else if (leg == LEG_RR) ampRr = clamped;
    }

    function applySpeed(speed: number): void {
        const s = clampVal(speed, 1, 10);
        currentSpeed = s;
        gaitStepDelay = 44 - s * 4;
    }

    function runCrawlGaitCycle(): void {
        const staggerMs = Math.idiv(gaitStepDelay, 5);
        const remMs = gaitStepDelay - staggerMs * 4;

        if (!isWalking || isCancelled()) {
            inGaitStance = false;
            return;
        }

        if (
            lastServo[1] != LIFT_FR_DOWN ||
            lastServo[3] != LIFT_RR_DOWN ||
            lastServo[5] != LIFT_FL_DOWN ||
            lastServo[7] != LIFT_RL_DOWN
        ) {
            const s1V = lastServo[1];
            const s3V = lastServo[3];
            const s5V = lastServo[5];
            const s7V = lastServo[7];
            for (let s = 1; s <= 6; s++) {
                if (!isWalking || isCancelled()) {
                    inGaitStance = false;
                    return;
                }
                driveServo(1, S2, interpCubic(s1V, LIFT_FR_DOWN, s), staggerMs);
                driveServo(3, S4, interpCubic(s3V, LIFT_RR_DOWN, s), staggerMs);
                driveServo(5, S6, interpCubic(s5V, LIFT_FL_DOWN, s), staggerMs);
                driveServo(7, S8, interpCubic(s7V, LIFT_RL_DOWN, s), staggerMs);
            }
            setServoExact(1, S2, LIFT_FR_DOWN, 0);
            setServoExact(3, S4, LIFT_RR_DOWN, 0);
            setServoExact(5, S6, LIFT_FL_DOWN, 0);
            setServoExact(7, S8, LIFT_RL_DOWN, 0);
        }

        const p1RlStart = BASE_RL_NEUTRAL + DIR_RL * ampRl;
        if (!inGaitStance && absVal(lastServo[6] - p1RlStart) >= 4) {
            const rlS0 = lastServo[6];
            for (let s = 1; s <= 6; s++) {
                if (!isWalking || isCancelled()) {
                    inGaitStance = false;
                    return;
                }
                driveServo(7, S8, calcSwingLift(LIFT_RL_DOWN, Math.idiv(LIFT_H_RL, 2), s), staggerMs);
                driveServo(6, S7, interpCubic(rlS0, p1RlStart, s), staggerMs * 2);
            }
            setServoExact(7, S8, LIFT_RL_DOWN, 0);
        }
        inGaitStance = true;

        // PHASE 1: FL Moves
        let sFl = lastServo[4];
        let sRl = lastServo[6];
        let sFr = lastServo[0];
        let sRr = lastServo[2];
        let eFl = BASE_FL_NEUTRAL + DIR_FL * ampFl;
        let eRl = BASE_RL_NEUTRAL + DIR_RL * calcStanceOffset(0, 6, ampRl);
        let eFr = BASE_FR_NEUTRAL + DIR_FR * calcStanceOffset(1, 6, ampFr);
        let eRr = BASE_RR_NEUTRAL + DIR_RR * calcStanceOffset(2, 6, ampRr);

        for (let s = 1; s <= 6; s++) {
            if (!isWalking || isCancelled()) { inGaitStance = false; return; }
            driveServo(5, S6, calcSwingLift(LIFT_FL_DOWN, LIFT_H_FL, s), staggerMs);
            driveServo(4, S5, interpCubic(sFl, eFl, s), staggerMs);
            driveServo(6, S7, interpLinear(sRl, eRl, s), staggerMs);
            driveServo(0, S1, interpLinear(sFr, eFr, s), staggerMs);
            driveServo(2, S3, interpLinear(sRr, eRr, s), remMs);
        }
        setServoExact(5, S6, LIFT_FL_DOWN, 0);

        // PHASE 2: RR Moves
        sRr = lastServo[2];
        sFl = lastServo[4];
        sRl = lastServo[6];
        sFr = lastServo[0];
        eRr = BASE_RR_NEUTRAL + DIR_RR * ampRr;
        eFl = BASE_FL_NEUTRAL + DIR_FL * calcStanceOffset(0, 6, ampFl);
        eRl = BASE_RL_NEUTRAL + DIR_RL * calcStanceOffset(1, 6, ampRl);
        eFr = BASE_FR_NEUTRAL + DIR_FR * calcStanceOffset(2, 6, ampFr);

        for (let s = 1; s <= 6; s++) {
            if (!isWalking || isCancelled()) { inGaitStance = false; return; }
            driveServo(3, S4, calcSwingLift(LIFT_RR_DOWN, LIFT_H_RR, s), staggerMs);
            driveServo(2, S3, interpCubic(sRr, eRr, s), staggerMs);
            driveServo(4, S5, interpLinear(sFl, eFl, s), staggerMs);
            driveServo(6, S7, interpLinear(sRl, eRl, s), staggerMs);
            driveServo(0, S1, interpLinear(sFr, eFr, s), remMs);
        }
        setServoExact(3, S4, LIFT_RR_DOWN, 0);

        // PHASE 3: FR Moves
        sFr = lastServo[0];
        sRr = lastServo[2];
        sFl = lastServo[4];
        sRl = lastServo[6];
        eFr = BASE_FR_NEUTRAL + DIR_FR * ampFr;
        eRr = BASE_RR_NEUTRAL + DIR_RR * calcStanceOffset(0, 6, ampRr);
        eFl = BASE_FL_NEUTRAL + DIR_FL * calcStanceOffset(1, 6, ampFl);
        eRl = BASE_RL_NEUTRAL + DIR_RL * calcStanceOffset(2, 6, ampRl);

        for (let s = 1; s <= 6; s++) {
            if (!isWalking || isCancelled()) { inGaitStance = false; return; }
            driveServo(1, S2, calcSwingLift(LIFT_FR_DOWN, LIFT_H_FR, s), staggerMs);
            driveServo(0, S1, interpCubic(sFr, eFr, s), staggerMs);
            driveServo(2, S3, interpLinear(sRr, eRr, s), staggerMs);
            driveServo(4, S5, interpLinear(sFl, eFl, s), staggerMs);
            driveServo(6, S7, interpLinear(sRl, eRl, s), remMs);
        }
        setServoExact(1, S2, LIFT_FR_DOWN, 0);

        // PHASE 4: RL Moves
        sRl = lastServo[6];
        sFr = lastServo[0];
        sRr = lastServo[2];
        sFl = lastServo[4];
        eRl = BASE_RL_NEUTRAL + DIR_RL * ampRl;
        eFr = BASE_FR_NEUTRAL + DIR_FR * calcStanceOffset(0, 6, ampFr);
        eRr = BASE_RR_NEUTRAL + DIR_RR * calcStanceOffset(1, 6, ampRr);
        eFl = BASE_FL_NEUTRAL + DIR_FL * calcStanceOffset(2, 6, ampFl);

        for (let s = 1; s <= 6; s++) {
            if (!isWalking || isCancelled()) { inGaitStance = false; return; }
            driveServo(7, S8, calcSwingLift(LIFT_RL_DOWN, LIFT_H_RL, s), staggerMs);
            driveServo(6, S7, interpCubic(sRl, eRl, s), staggerMs);
            driveServo(0, S1, interpLinear(sFr, eFr, s), staggerMs);
            driveServo(2, S3, interpLinear(sRr, eRr, s), staggerMs);
            driveServo(4, S5, interpLinear(sFl, eFl, s), remMs);
        }
        setServoExact(7, S8, LIFT_RL_DOWN, 0);
    }

    function enterStand(): boolean {
        inGaitStance = false;
        const standToken = activeToken;
        if (servoInitialized) {
            if (actionToken != standToken || isCancelled()) return false;
            if (!recoverToStance()) return false;
        }

        setServoExact(1, S2, LIFT_FR_REST, 20);
        setServoExact(3, S4, LIFT_RR_REST, 20);
        setServoExact(5, S6, LIFT_FL_REST, 20);
        setServoExact(7, S8, LIFT_RL_REST, 40);
        if (servoInitialized && (actionToken != standToken || isCancelled())) return false;

        setServoExact(0, S1, BASE_FR_NEUTRAL, 20);
        setServoExact(2, S3, BASE_RR_NEUTRAL, 20);
        setServoExact(4, S5, BASE_FL_NEUTRAL, 20);
        setServoExact(6, S7, BASE_RL_NEUTRAL, 40);
        if (servoInitialized && (actionToken != standToken || isCancelled())) return false;

        // 1 deg torque-relief nudge
        writeServo270(S2, clampServoAngle(1, LIFT_FR_REST + 1));
        basic.pause(10);
        if (servoInitialized && (actionToken != standToken || isCancelled())) return false;
        writeServo270(S4, clampServoAngle(3, LIFT_RR_REST + 1));
        basic.pause(10);
        if (servoInitialized && (actionToken != standToken || isCancelled())) return false;
        writeServo270(S6, clampServoAngle(5, LIFT_FL_REST - 1));
        basic.pause(10);
        if (servoInitialized && (actionToken != standToken || isCancelled())) return false;
        writeServo270(S8, clampServoAngle(7, LIFT_RL_REST - 1));
        basic.pause(25);
        if (servoInitialized && (actionToken != standToken || isCancelled())) return false;

        setServoExact(1, S2, LIFT_FR_REST, 8);
        setServoExact(3, S4, LIFT_RR_REST, 8);
        setServoExact(5, S6, LIFT_FL_REST, 8);
        setServoExact(7, S8, LIFT_RL_REST, 8);
        if (servoInitialized && (actionToken != standToken || isCancelled())) return false;

        lastServo[0] = BASE_FR_NEUTRAL;
        lastServo[1] = LIFT_FR_REST;
        lastServo[2] = BASE_RR_NEUTRAL;
        lastServo[3] = LIFT_RR_REST;
        lastServo[4] = BASE_FL_NEUTRAL;
        lastServo[5] = LIFT_FL_REST;
        lastServo[6] = BASE_RL_NEUTRAL;
        lastServo[7] = LIFT_RL_REST;
        levelPitchCmd = 0;
        levelRollCmd = 0;
        crouchFr = 0;
        crouchRr = 0;
        crouchFl = 0;
        crouchRl = 0;
        servoInitialized = true;
        return true;
    }

    // --------------------------------------------------------------
    // Wave, Dance & Self-Leveling Implementations
    // --------------------------------------------------------------

    function runWave(): void {
        inGaitStance = false;
        if (isCancelled()) return;

        // 1. Step Rear Right forward under middle of right side
        const s3Start = lastServo[2];
        for (let s = 1; s <= 6; s++) {
            if (isCancelled()) return;
            driveServo(3, S4, calcSwingLift(LIFT_RR_DOWN, 55, s), 8);
            driveServo(2, S3, interpCubic(s3Start, 168, s), 12);
        }
        setServoExact(3, S4, 80, 15);
        setServoExact(2, S3, 168, 15);

        // 2. Tilt weight back-left
        const s6Start = lastServo[5];
        const s8Start = lastServo[7];
        for (let s = 1; s <= 6; s++) {
            if (isCancelled()) return;
            driveServo(5, S6, interpCubic(s6Start, 265, s), 6);
            driveServo(7, S8, interpCubic(s8Start, 205, s), 12);
        }

        // 3. Lift Front Right and wave 3 times
        const s1Start = lastServo[0];
        const s2Start = lastServo[1];
        for (let s = 1; s <= 6; s++) {
            if (isCancelled()) return;
            driveServo(1, S2, interpCubic(s2Start, 220, s), 10);
            driveServo(0, S1, interpCubic(s1Start, 225, s), 14);
        }

        for (let i = 0; i < 3; i++) {
            if (isCancelled()) return;
            const s1A = lastServo[0];
            const s2A = lastServo[1];
            for (let s = 1; s <= 6; s++) {
                if (isCancelled()) return;
                driveServo(0, S1, interpCubic(s1A, 180, s), 8);
                driveServo(1, S2, interpCubic(s2A, 200, s), 14);
            }
            const s1B = lastServo[0];
            const s2B = lastServo[1];
            for (let s = 1; s <= 6; s++) {
                if (isCancelled()) return;
                driveServo(0, S1, interpCubic(s1B, 225, s), 8);
                driveServo(1, S2, interpCubic(s2B, 240, s), 14);
            }
        }

        // 4. Plant Front Right firmly
        const s1End = lastServo[0];
        const s2End = lastServo[1];
        for (let s = 1; s <= 6; s++) {
            if (isCancelled()) return;
            driveServo(0, S1, interpCubic(s1End, BASE_FR_NEUTRAL, s), 10);
            driveServo(1, S2, interpCubic(s2End, 80, s), 18);
        }
        setServoExact(0, S1, BASE_FR_NEUTRAL, 10);
        setServoExact(1, S2, 80, 30);

        // 5. Level left side
        const s6Cur = lastServo[5];
        const s8Cur = lastServo[7];
        for (let s = 1; s <= 6; s++) {
            if (isCancelled()) return;
            driveServo(5, S6, interpCubic(s6Cur, LIFT_FL_REST, s), 8);
            driveServo(7, S8, interpCubic(s8Cur, LIFT_RL_REST, s), 14);
        }
        setServoExact(5, S6, LIFT_FL_REST, 10);
        setServoExact(7, S8, LIFT_RL_REST, 20);

        // 6. Return Rear Right
        const s3Cur = lastServo[2];
        for (let s = 1; s <= 6; s++) {
            if (isCancelled()) return;
            driveServo(3, S4, calcSwingLift(LIFT_RR_REST, 55, s), 8);
            driveServo(2, S3, interpCubic(s3Cur, BASE_RR_NEUTRAL, s), 12);
        }
        setServoExact(3, S4, LIFT_RR_REST, 10);
        setServoExact(2, S3, BASE_RR_NEUTRAL, 10);

        if (!isCancelled()) {
            enterStand();
        }
    }

    function danceSingleLeg(
        hIdx: number,
        hPort: number,
        vIdx: number,
        vPort: number,
        baseNeutral: number,
        dirH: number,
        vDown: number,
        liftH: number
    ): void {
        const staggerMs = 8;
        const stepWait = 20;
        const halfLift = Math.idiv(liftH, 2);

        for (let s = 1; s <= 6; s++) {
            if (!isDancing || isCancelled()) return;
            driveServo(vIdx, vPort, calcSwingLift(vDown, halfLift, s), staggerMs);
            driveServo(hIdx, hPort, baseNeutral + dirH * interpCubic(0, -DANCE_H_AMP, s), stepWait);
        }
        for (let s = 1; s <= 6; s++) {
            if (!isDancing || isCancelled()) return;
            driveServo(vIdx, vPort, calcSwingLift(vDown, liftH, s), staggerMs);
            driveServo(hIdx, hPort, baseNeutral + dirH * calcSwingOffset(s, DANCE_H_AMP), stepWait);
        }
        for (let s = 1; s <= 6; s++) {
            if (!isDancing || isCancelled()) return;
            driveServo(vIdx, vPort, calcSwingLift(vDown, liftH, s), staggerMs);
            driveServo(hIdx, hPort, baseNeutral + dirH * calcSwingOffset(s, -DANCE_H_AMP), stepWait);
        }
        for (let s = 1; s <= 6; s++) {
            if (!isDancing || isCancelled()) return;
            driveServo(vIdx, vPort, calcSwingLift(vDown, halfLift, s), staggerMs);
            driveServo(hIdx, hPort, baseNeutral + dirH * interpCubic(-DANCE_H_AMP, 0, s), stepWait);
        }

        setServoExact(vIdx, vPort, vDown, 10);
        setServoExact(hIdx, hPort, baseNeutral, 15);
    }

    function runDance(): void {
        inGaitStance = false;
        isDancing = true;

        if (!enterStand() || pauseInterruptible(250)) {
            isDancing = false;
            return;
        }

        if (isDancing && !isCancelled()) {
            danceSingleLeg(4, S5, 5, S6, BASE_FL_NEUTRAL, DIR_FL, LIFT_FL_DOWN, LIFT_H_FL);
        }
        if (isDancing && !isCancelled()) {
            danceSingleLeg(0, S1, 1, S2, BASE_FR_NEUTRAL, DIR_FR, LIFT_FR_DOWN, LIFT_H_FR);
        }
        if (isDancing && !isCancelled()) {
            danceSingleLeg(6, S7, 7, S8, BASE_RL_NEUTRAL, DIR_RL, LIFT_RL_DOWN, LIFT_H_RL);
        }
        if (isDancing && !isCancelled()) {
            danceSingleLeg(2, S3, 3, S4, BASE_RR_NEUTRAL, DIR_RR, LIFT_RR_DOWN, LIFT_H_RR);
        }

        if (isDancing && !isCancelled()) {
            const lDip = LIFT_FL_DOWN - DANCE_V_AMP;
            const rDip = LIFT_FR_DOWN + DANCE_V_AMP;

            for (let s = 1; s <= 6; s++) {
                if (!isDancing || isCancelled()) break;
                const lV = interpCubic(LIFT_FL_DOWN, lDip, s);
                driveServo(5, S6, lV, 6);
                driveServo(7, S8, lV, 10);
            }

            for (let sway = 0; sway < 2; sway++) {
                if (!isDancing || isCancelled()) break;
                for (let s = 1; s <= 6; s++) {
                    if (!isDancing || isCancelled()) break;
                    const lV = interpCubic(lDip, LIFT_FL_DOWN, s);
                    const rV = interpCubic(LIFT_FR_DOWN, rDip, s);
                    driveServo(1, S2, rV, 5);
                    driveServo(3, S4, rV, 5);
                    driveServo(5, S6, lV, 5);
                    driveServo(7, S8, lV, 9);
                }
                if (!isDancing || isCancelled()) break;
                for (let s = 1; s <= 6; s++) {
                    if (!isDancing || isCancelled()) break;
                    const lV = interpCubic(LIFT_FL_DOWN, lDip, s);
                    const rV = interpCubic(rDip, LIFT_FR_DOWN, s);
                    driveServo(5, S6, lV, 5);
                    driveServo(7, S8, lV, 5);
                    driveServo(1, S2, rV, 5);
                    driveServo(3, S4, rV, 9);
                }
            }

            for (let s = 1; s <= 6; s++) {
                if (!isDancing || isCancelled()) break;
                const lV = interpCubic(lDip, LIFT_FL_DOWN, s);
                driveServo(5, S6, lV, 6);
                driveServo(7, S8, lV, 10);
            }
        }

        if (isDancing && !isCancelled()) {
            // Celebration pose & waves
            for (let s = 1; s <= 6; s++) {
                if (!isDancing || isCancelled()) break;
                driveServo(3, S4, interpCubic(LIFT_RR_DOWN, LIFT_RR_DOWN + DANCE_V_AMP, s), 6);
                driveServo(7, S8, interpCubic(LIFT_RL_DOWN, LIFT_RL_DOWN - DANCE_V_AMP, s), 10);
            }

            for (let s = 1; s <= 6; s++) {
                if (!isDancing || isCancelled()) break;
                driveServo(1, S2, interpCubic(LIFT_FR_DOWN, LIFT_FR_DOWN + LIFT_H_FR, s), 6);
                driveServo(0, S1, BASE_FR_NEUTRAL + DIR_FR * interpCubic(0, -DANCE_H_AMP, s), 10);
            }
            for (let s = 1; s <= 6; s++) {
                if (!isDancing || isCancelled()) break;
                driveServo(0, S1, BASE_FR_NEUTRAL + DIR_FR * calcSwingOffset(s, DANCE_H_AMP), 12);
            }
            for (let s = 1; s <= 6; s++) {
                if (!isDancing || isCancelled()) break;
                driveServo(0, S1, BASE_FR_NEUTRAL + DIR_FR * interpCubic(DANCE_H_AMP, 0, s), 6);
                driveServo(1, S2, interpCubic(LIFT_FR_DOWN + LIFT_H_FR, LIFT_FR_DOWN, s), 10);
            }
            setServoExact(1, S2, LIFT_FR_DOWN, 5);
            setServoExact(0, S1, BASE_FR_NEUTRAL, 5);

            for (let s = 1; s <= 6; s++) {
                if (!isDancing || isCancelled()) break;
                driveServo(5, S6, interpCubic(LIFT_FL_DOWN, LIFT_FL_DOWN + LIFT_H_FL, s), 6);
                driveServo(4, S5, BASE_FL_NEUTRAL + DIR_FL * interpCubic(0, -DANCE_H_AMP, s), 10);
            }
            for (let s = 1; s <= 6; s++) {
                if (!isDancing || isCancelled()) break;
                driveServo(4, S5, BASE_FL_NEUTRAL + DIR_FL * calcSwingOffset(s, DANCE_H_AMP), 12);
            }
            for (let s = 1; s <= 6; s++) {
                if (!isDancing || isCancelled()) break;
                driveServo(4, S5, BASE_FL_NEUTRAL + DIR_FL * interpCubic(DANCE_H_AMP, 0, s), 6);
                driveServo(5, S6, interpCubic(LIFT_FL_DOWN + LIFT_H_FL, LIFT_FL_DOWN, s), 10);
            }
            setServoExact(5, S6, LIFT_FL_DOWN, 5);
            setServoExact(4, S5, BASE_FL_NEUTRAL, 5);

            for (let s = 1; s <= 6; s++) {
                if (!isDancing || isCancelled()) break;
                driveServo(3, S4, interpCubic(LIFT_RR_DOWN + DANCE_V_AMP, LIFT_RR_DOWN, s), 6);
                driveServo(7, S8, interpCubic(LIFT_RL_DOWN - DANCE_V_AMP, LIFT_RL_DOWN, s), 10);
            }
        }

        if (isDancing && !isCancelled()) {
            isDancing = false;
            enterStand();
        } else {
            isDancing = false;
        }
    }

    function calibrateLevelRefInternal(): boolean {
        let zSum = 0;
        let xSum = 0;
        for (let i = 0; i < 8; i++) {
            if (isCancelled()) return false;
            zSum += input.acceleration(Dimension.Z);
            xSum += input.acceleration(Dimension.X);
            if (pauseInterruptible(15)) return false;
        }
        levelPitchZero = Math.idiv(zSum, 8);
        levelRollZero = Math.idiv(xSum, 8);
        levelPitchFilt10 = 0;
        levelRollFilt10 = 0;
        levelPitchCmd = 0;
        levelRollCmd = 0;
        crouchFr = 0;
        crouchRr = 0;
        crouchFl = 0;
        crouchRl = 0;
        levelCalibrated = true;
        return true;
    }

    function stepSelfLevel(): boolean {
        if (!isLeveling || isCancelled()) return false;

        let dzSum = 0;
        let dxSum = 0;
        for (let i = 0; i < 3; i++) {
            dzSum += levelPitchZero - input.acceleration(Dimension.Z);
            dxSum += levelRollZero - input.acceleration(Dimension.X);
            basic.pause(4);
            if (!isLeveling || isCancelled()) return false;
        }

        const pSample10 = Math.idiv(dzSum * 191, 1024) * LEVEL_PITCH_SIGN;
        const rSample10 = Math.idiv(dxSum * 191, 1024) * LEVEL_ROLL_SIGN;

        levelPitchFilt10 = Math.idiv(LEVEL_FILTER_ALPHA_100 * pSample10 + (100 - LEVEL_FILTER_ALPHA_100) * levelPitchFilt10, 100);
        levelRollFilt10 = Math.idiv(LEVEL_FILTER_ALPHA_100 * rSample10 + (100 - LEVEL_FILTER_ALPHA_100) * levelRollFilt10, 100);

        let pitchErr = Math.idiv(levelPitchFilt10, 10);
        let rollErr = Math.idiv(levelRollFilt10, 10);

        if (absVal(pitchErr) <= LEVEL_DEADBAND_DEG) pitchErr = 0;
        if (absVal(rollErr) <= LEVEL_DEADBAND_DEG) rollErr = 0;

        if (pitchErr == 0 && rollErr == 0) {
            return !pauseInterruptible(LEVEL_SETTLE_MS);
        }

        let dp = Math.idiv(pitchErr * LEVEL_KP_100, 100);
        if (dp == 0 && pitchErr != 0) dp = pitchErr > 0 ? 1 : -1;
        dp = clampVal(dp, -3, 3);

        let dr = Math.idiv(rollErr * LEVEL_KP_100, 100);
        if (dr == 0 && rollErr != 0) dr = rollErr > 0 ? 1 : -1;
        dr = clampVal(dr, -3, 3);

        const maxPlanar = Math.idiv(MAX_LEVEL_CROUCH, 2);
        levelPitchCmd = clampVal(levelPitchCmd + dp, -maxPlanar, maxPlanar);
        levelRollCmd = clampVal(levelRollCmd + dr, -maxPlanar, maxPlanar);

        let nf = levelPitchCmd - levelRollCmd;
        let nr = -levelPitchCmd - levelRollCmd;
        let nlf = levelPitchCmd + levelRollCmd;
        let nlr = -levelPitchCmd + levelRollCmd;

        let minC = nf;
        if (nr < minC) minC = nr;
        if (nlf < minC) minC = nlf;
        if (nlr < minC) minC = nlr;

        nf = clampVal(nf - minC, 0, MAX_LEVEL_CROUCH);
        nr = clampVal(nr - minC, 0, MAX_LEVEL_CROUCH);
        nlf = clampVal(nlf - minC, 0, MAX_LEVEL_CROUCH);
        nlr = clampVal(nlr - minC, 0, MAX_LEVEL_CROUCH);

        crouchFr = clampVal(nf, crouchFr - MAX_LEVEL_STEP, crouchFr + MAX_LEVEL_STEP);
        crouchRr = clampVal(nr, crouchRr - MAX_LEVEL_STEP, crouchRr + MAX_LEVEL_STEP);
        crouchFl = clampVal(nlf, crouchFl - MAX_LEVEL_STEP, crouchFl + MAX_LEVEL_STEP);
        crouchRl = clampVal(nlr, crouchRl - MAX_LEVEL_STEP, crouchRl + MAX_LEVEL_STEP);

        const targetFr = clampServoAngle(1, LIFT_FR_REST + crouchFr);
        const targetRr = clampServoAngle(3, LIFT_RR_REST + crouchRr);
        const targetFl = clampServoAngle(5, LIFT_FL_REST - crouchFl);
        const targetRl = clampServoAngle(7, LIFT_RL_REST - crouchRl);

        setServoExact(1, S2, targetFr, 6);
        setServoExact(3, S4, targetRr, 6);
        setServoExact(5, S6, targetFl, 6);
        setServoExact(7, S8, targetRl, 6);

        return !pauseInterruptible(LEVEL_SETTLE_MS);
    }

    function runSelfLevelMode(): void {
        if (!enterStand()) {
            isLeveling = false;
            return;
        }
        if (pauseInterruptible(400)) {
            isLeveling = false;
            return;
        }
        if (!levelCalibrated) {
            if (!calibrateLevelRefInternal()) {
                isLeveling = false;
                return;
            }
        } else {
            levelPitchFilt10 = 0;
            levelRollFilt10 = 0;
            levelPitchCmd = 0;
            levelRollCmd = 0;
        }

        while (isLeveling && !isCancelled() && !modeIsTimedOut()) {
            if (!stepSelfLevel()) break;
        }
        if (modeIsTimedOut()) {
            modeEndTimeMs = 0;
        }
        isLeveling = false;
    }

    // --------------------------------------------------------------
    // Single-Fiber Mode Dispatcher
    // --------------------------------------------------------------

    function configureGaitForMode(mode: CrabBotMode): void {
        applySpeed(currentSpeed);
        if (mode == CrabBotMode.WalkForward) {
            setLegAmplitude(LEG_FL, SWING_AMP);
            setLegAmplitude(LEG_RL, SWING_AMP);
            setLegAmplitude(LEG_FR, SWING_AMP);
            setLegAmplitude(LEG_RR, SWING_AMP);
        } else if (mode == CrabBotMode.WalkBackward) {
            setLegAmplitude(LEG_FL, -SWING_AMP);
            setLegAmplitude(LEG_RL, -SWING_AMP);
            setLegAmplitude(LEG_FR, -SWING_AMP);
            setLegAmplitude(LEG_RR, -SWING_AMP);
        } else if (mode == CrabBotMode.TurnLeft) {
            const k = clampVal(absVal(currentTurnPct), 0, MAX_TURN_PCT);
            const aLeft = Math.idiv(SWING_AMP * (100 - k), 100);
            const aRight = Math.idiv(SWING_AMP * (100 + k), 100);
            setLegAmplitude(LEG_FL, aLeft);
            setLegAmplitude(LEG_RL, aLeft);
            setLegAmplitude(LEG_FR, aRight);
            setLegAmplitude(LEG_RR, aRight);
        } else if (mode == CrabBotMode.TurnRight) {
            const k = -clampVal(absVal(currentTurnPct), 0, MAX_TURN_PCT);
            const aLeft = Math.idiv(SWING_AMP * (100 - k), 100);
            const aRight = Math.idiv(SWING_AMP * (100 + k), 100);
            setLegAmplitude(LEG_FL, aLeft);
            setLegAmplitude(LEG_RL, aLeft);
            setLegAmplitude(LEG_FR, aRight);
            setLegAmplitude(LEG_RR, aRight);
        } else if (mode == CrabBotMode.SpinLeft) {
            setLegAmplitude(LEG_FL, -SWING_AMP);
            setLegAmplitude(LEG_RL, -SWING_AMP);
            setLegAmplitude(LEG_FR, SWING_AMP);
            setLegAmplitude(LEG_RR, SWING_AMP);
        } else if (mode == CrabBotMode.SpinRight) {
            setLegAmplitude(LEG_FL, SWING_AMP);
            setLegAmplitude(LEG_RL, SWING_AMP);
            setLegAmplitude(LEG_FR, -SWING_AMP);
            setLegAmplitude(LEG_RR, -SWING_AMP);
        }
    }

    function modeIsTimedOut(): boolean {
        return modeEndTimeMs > 0 && input.runningTime() >= modeEndTimeMs;
    }

    function ensureDispatcher(): void {
        if (dispatcherStarted) return;
        dispatcherStarted = true;
        control.inBackground(() => {
            while (true) {
                if (hasPendingMode) {
                    hasPendingMode = false;
                    const mode = pendingMode;
                    activeMode = mode;
                    activeToken = actionToken;

                    if (mode == CrabBotMode.Stand || mode == CrabBotMode.Stop) {
                        if (recoverToStance() && !isCancelled()) {
                            enterStand();
                        }
                        activeMode = CrabBotMode.Stand;
                    } else if (
                        mode == CrabBotMode.WalkForward ||
                        mode == CrabBotMode.WalkBackward ||
                        mode == CrabBotMode.TurnLeft ||
                        mode == CrabBotMode.TurnRight ||
                        mode == CrabBotMode.SpinLeft ||
                        mode == CrabBotMode.SpinRight
                    ) {
                        if (recoverToStance() && !isCancelled()) {
                            isWalking = true;
                            configureGaitForMode(mode);
                            while (isWalking && !isCancelled() && !modeIsTimedOut()) {
                                runCrawlGaitCycle();
                            }
                            isWalking = false;
                            if (!isCancelled() && modeIsTimedOut()) {
                                modeEndTimeMs = 0;
                                if (recoverToStance() && !isCancelled()) {
                                    enterStand();
                                }
                                activeMode = CrabBotMode.Stand;
                            }
                        }
                    } else if (mode == CrabBotMode.Wave) {
                        if (recoverToStance() && !isCancelled()) {
                            runWave();
                        }
                        if (!isCancelled()) {
                            activeMode = CrabBotMode.Stand;
                        }
                    } else if (mode == CrabBotMode.Dance) {
                        if (recoverToStance() && !isCancelled()) {
                            runDance();
                        }
                        if (!isCancelled()) {
                            activeMode = CrabBotMode.Stand;
                        }
                    } else if (mode == CrabBotMode.SelfBalance) {
                        if (recoverToStance() && !isCancelled()) {
                            isLeveling = true;
                            runSelfLevelMode();
                        }
                        if (!isCancelled()) {
                            enterStand();
                            activeMode = CrabBotMode.Stand;
                        }
                    }
                } else {
                    basic.pause(20);
                }
            }
        });
    }

    function queueMode(mode: CrabBotMode, durationMs: number): void {
        ensureDispatcher();
        actionToken += 1;
        isWalking = false;
        isDancing = false;
        isLeveling = false;
        modeEndTimeMs = durationMs > 0 ? input.runningTime() + durationMs : 0;
        pendingMode = mode;
        hasPendingMode = true;
    }

    // --------------------------------------------------------------
    // Public MakeCode Blocks
    // --------------------------------------------------------------

    /**
     * Switch CrabBot Mini to a selectable action mode.
     */
    //% blockId=CrabBotMini_setMode block="CrabBot set mode %mode"
    //% weight=100
    //% blockGap=10
    export function setMode(mode: CrabBotMode): void {
        queueMode(mode, 0);
    }

    /**
     * Run a selectable CrabBot Mini mode for a duration in milliseconds, then return to stand.
     */
    //% blockId=CrabBotMini_runModeFor block="CrabBot run mode %mode|for %durationMs ms"
    //% weight=95
    //% durationMs.defl=3000 durationMs.min=200 durationMs.max=60000
    //% blockGap=10
    export function runModeFor(mode: CrabBotMode, durationMs: number): void {
        queueMode(mode, durationMs);
    }

    /**
     * Set walking/turning speed (1 = slow, 5 = normal, 10 = fast).
     */
    //% blockId=CrabBotMini_setSpeed block="CrabBot set speed %speed"
    //% weight=90
    //% speed.min=1 speed.max=10 speed.defl=5
    //% blockGap=10
    export function setSpeed(speed: number): void {
        applySpeed(speed);
    }

    /**
     * Turn while walking forward using differential stride strength (0 to 70%).
     */
    //% blockId=CrabBotMini_turn block="CrabBot turn %direction|strength %strengthPct \\%"
    //% weight=85
    //% strengthPct.min=0 strengthPct.max=70 strengthPct.defl=40
    //% blockGap=10
    export function turn(direction: CrabBotTurn, strengthPct: number): void {
        currentTurnPct = clampVal(absVal(strengthPct), 0, MAX_TURN_PCT);
        queueMode(direction == CrabBotTurn.Left ? CrabBotMode.TurnLeft : CrabBotMode.TurnRight, 0);
    }

    /**
     * Spin in place left or right at the given speed (1..10).
     */
    //% blockId=CrabBotMini_turnInPlace block="CrabBot spin %direction|speed %speed"
    //% weight=80
    //% speed.min=1 speed.max=10 speed.defl=5
    //% blockGap=10
    export function turnInPlace(direction: CrabBotTurn, speed: number): void {
        applySpeed(speed);
        queueMode(direction == CrabBotTurn.Left ? CrabBotMode.SpinLeft : CrabBotMode.SpinRight, 0);
    }

    /**
     * Return CrabBot Mini to neutral standing X-stance.
     */
    //% blockId=CrabBotMini_stand block="CrabBot stand"
    //% weight=75
    //% blockGap=10
    export function stand(): void {
        queueMode(CrabBotMode.Stand, 0);
    }

    /**
     * Stop any active CrabBot Mini movement and recover safely to standing stance.
     */
    //% blockId=CrabBotMini_stop block="CrabBot stop"
    //% weight=70
    //% blockGap=10
    export function stop(): void {
        queueMode(CrabBotMode.Stop, 0);
    }

    /**
     * Calibrate the flat-surface accelerometer reference for Self Balance mode.
     */
    //% blockId=CrabBotMini_calibrateBalance block="CrabBot calibrate self balance"
    //% weight=65
    //% blockGap=10
    export function calibrateBalance(): void {
        levelCalibrated = false;
        calibrateLevelRefInternal();
    }

    /**
     * Returns the currently active CrabBot Mini mode.
     */
    //% blockId=CrabBotMini_currentMode block="CrabBot current mode"
    //% weight=60
    export function currentMode(): CrabBotMode {
        return activeMode;
    }
}
