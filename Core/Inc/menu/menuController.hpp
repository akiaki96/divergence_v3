#pragma once

#include "menu/menu.hpp"
#include <boost/container/static_vector.hpp>


class MenuController {
public:
    MenuController();

    MenuNode* next();
    MenuNode* prev();
    MenuNode* enter();
    MenuNode* back();

    MenuNode* selected() const;

private:
    Menu menu_;

    MenuNode* currentMenuNode_ = menu_.root();
    uint8_t index_;
    uint8_t depth_;

};