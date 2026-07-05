#pragma once

#include "menu/menu.hpp"
#include <boost/container/static_vector.hpp>


class MenuController {
public:
    MenuController();

    void next();
    void prev();
    void enter();
    void back();

    void setChild(uint8_t index);

    const MenuNode* currentMenuNode() const;
    void currentInfo() const;
    void tree() const;

    uint8_t index() const;

private:
    Menu menu_;

    const MenuNode* currentMenuNode_ = menu_.root();
    uint8_t index_;
    uint8_t depth_;

};