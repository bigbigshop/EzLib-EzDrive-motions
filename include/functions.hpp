#pragma once

// 定義狀態
enum class IntakeState {
    STOP,
    INTAKE,
    L1,
    L2,
    R2,
    REVERSE,
    REVERSING,
    ALL_INTAKE,
    AUTON_INTAKE,
    AUTON_EATL1,
};

void pneumaticsBtn();

// FSM 相關函數
void setIntakeState(IntakeState state);
void updateIntakeFSM();