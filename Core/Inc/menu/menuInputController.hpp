#pragma once

#include "menu/menuController.hpp"

class MenuInputController {
public:
    explicit MenuInputController(MenuController& menuController);

    void syncUpdate();
    void asyncUpdate();

    void enable();
    void disable();

private:
    void lock(uint32_t delay);

    MenuController& controller_;   // menuInstance.cpp の menuController（コピーするとメニュー全体が2つになる）

    uint32_t lockUntil_ = 0; // ms
    float encoderDistance_ = 0.0f;

    bool lock_ = false;
    // メニューの操作中（ハンドラの実行が終わってから次のハンドラが始まるまで）。
    // この間は毎tick右モータのdutyを0にして，選択の合図（こりこり）を止める
    volatile bool selecting_ = false;

    bool isOnSelected_ = false;
    bool isOnEnter_ = false;
    MenuAction nowOnSelected = nullptr;
    MenuAction nowOnEntered = nullptr;
    
};