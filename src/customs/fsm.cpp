#include "main.h"
#include "customs/fsm.hpp"

IntakeState currentIntakeState = IntakeState::STOP;
IntakeState prevState = IntakeState::STOP; // 用於恢復狀態

int jamTimer = 0;
int reverseTimer = 0;
const int JAM_THRESHOLD = 2500; // mA
const int REVERSE_DURATION = 200; // 200ms

void setIntakeState(IntakeState state) {
    if (currentIntakeState != IntakeState::REVERSING) {
        currentIntakeState = state;
    }
}

void updateIntakeFSM() {
    // 監控馬達電流 (取最大值)
    double maxCurrent = std::max({
        low_roller.get_current_draw(),
        high_roller.get_current_draw(),
        intake_motors.get_current_draw()
    });

    switch (currentIntakeState) {
        case IntakeState::STOP:
            intake_motors.move(0);
            low_roller.move(0);
            high_roller.move(0);
            jamTimer = 0;
            break;

        case IntakeState::INTAKE:
        case IntakeState::L1:
        case IntakeState::L2:
            // 執行原本的馬達控制邏輯
            if (currentIntakeState == IntakeState::INTAKE) {
                if(matchload.is_extended()){
                    high_roller.move(75);
                    intake_motors.move(0);
                }else{
                    high_roller.move(50);
                    intake_motors.move(127);
                }
                low_roller.move(127);
                cover.set_value(false);
            } else if (currentIntakeState == IntakeState::L1) {
                intake_motors.move(0);
                low_roller.move(127);
                high_roller.move(75);
                cover.set_value(true);
            } else if (currentIntakeState == IntakeState::L2) {
                intake_motors.move(0);
                low_roller.move(127);
                high_roller.move(127);
                cover.set_value(true);
            } else if (currentIntakeState == IntakeState::R2) {
                intake_motors.move(-127);
                low_roller.move(-127);
                high_roller.move(-127);
                cover.set_value(true);
            }

            // --- Jam 檢測 ---
            if (maxCurrent > JAM_THRESHOLD) {
                jamTimer += 20;
            } else {
                jamTimer = 0;
            }

            if (jamTimer > 400) { // 如果卡住超過 400ms
                prevState = currentIntakeState; // 記住現在是 L1 還是 L2
                currentIntakeState = IntakeState::REVERSING;
                reverseTimer = 0;
                jamTimer = 0;
            }
            break;

        case IntakeState::REVERSING:
            // 反轉 500ms
            intake_motors.move(-100);
            low_roller.move(-127);
            high_roller.move(-127);
            
            reverseTimer += 20;
            if (reverseTimer >= REVERSE_DURATION) {
                currentIntakeState = prevState; // 500ms 後恢復之前的狀態 (L1/L2/INTAKE)
                reverseTimer = 0;
            }
            break;

        case IntakeState::REVERSE:
            left_intake.move(-70);
            right_intake.move(-80);
            low_roller.move(-127);
            high_roller.move(-127);
            break;
        case IntakeState::ALL_INTAKE:
            left_intake.move(127);
            right_intake.move(127);
            low_roller.move(127);
            high_roller.move(75);
            cover.set_value(true);
            break;
            
        case IntakeState::AUTON_INTAKE:
            left_intake.move(127);
            right_intake.move(127);
            low_roller.move(127);
            high_roller.move(100);
            cover.set_value(false);
            break;
        case IntakeState::AUTON_EATL1:
            left_intake.move(127);
            right_intake.move(127);
            low_roller.move(127);
            high_roller.move(0);
            cover.set_value(true);
            break;

        default:
            break;
    }
}