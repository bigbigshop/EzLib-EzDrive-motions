#include "main.h"
#include "customs/controls.hpp"

int hookSequenceTimer = 0;
bool hookSequenceActive = false;

void pneumaticsBtn(){
    if (controller.get_digital_new_press(DIGITAL_X)) {
        if(arm.is_extended()){
            hook.extend();
            arm.retract();
            hookSequenceActive = true;
            hookSequenceTimer = pros::millis(); // 記錄開始時間
        }else{
            arm.extend();
            hook.extend();
        }
    }
    if(hookSequenceActive && pros::millis() - hookSequenceTimer >= 400) {
        hook.retract();
        hookSequenceActive = false;
    }
    if(controller.get_digital_new_press(DIGITAL_A)) {
        hook.toggle();
    }
    if(controller.get_digital_new_press(DIGITAL_Y)) {
        matchload.toggle();
    }
}
