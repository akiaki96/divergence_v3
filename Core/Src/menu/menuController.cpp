#include "menu/menuController.hpp"
#include "common/debug.hpp"

MenuController::MenuController() {}

void MenuController::next() {
    index_ = (index_ + 1) % currentMenuNode_->childCount();
    currentMenuNode_ = currentMenuNode_->parent()->child(index_);
    currentMenuNode_->onSelected();
}

void MenuController::prev() {
    index_ = (index_ - 1 + currentMenuNode_->childCount()) % currentMenuNode_->childCount();
    currentMenuNode_ = currentMenuNode_->parent()->child(index_);
    currentMenuNode_->onSelected();
}

void MenuController::enter() {
    if (currentMenuNode_->isLeaf()) {
        currentMenuNode_->onEnter();
    } else if (currentMenuNode_ == nullptr) {
        LOG("currentMenuNode got nullptr\r\n");
        return;
    }

    ++depth_;
    currentMenuNode_ = currentMenuNode_->child(index_);
    currentMenuNode_->onSelected();
}

void MenuController::back() {
    if (currentMenuNode_ == currentMenuNode_->parent()) {
        LOG("Now on root\r\n");
    } else if (currentMenuNode_ == nullptr) {
        return;
    }

    --depth_;
    currentMenuNode_ = currentMenuNode_->parent();
    currentMenuNode_->onSelected();
}

const MenuNode* MenuController::currentMenuNode() const {
    return currentMenuNode_;
}

void MenuController::currentInfo() const {
    printf("=== Selector ===\n");
    printf("Current Menu: %s\n", currentMenuNode_->name());
    printf("Depth: %d\n", depth_);
    printf("================\n");
}

void MenuController::tree() const {
    menu_.tree();
}