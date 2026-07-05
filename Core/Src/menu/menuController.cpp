#include "menu/menuController.hpp"
#include "common/debug.hpp"

MenuController::MenuController() {}

void MenuController::next() {
    index_ = (index_ + 1) % currentMenuNode_->childCount();
    currentMenuNode_ = currentMenuNode_->parent()->child(index_);
}

void MenuController::prev() {
    index_ = (index_ + currentMenuNode_->childCount() - 1) % currentMenuNode_->childCount();
    currentMenuNode_ = currentMenuNode_->parent()->child(index_);
}

void MenuController::enter() {
    if (currentMenuNode_ == nullptr) {
        LOG("currentMenuNode got nullptr\r\n");
        return;
    }
    if (currentMenuNode_->isLeaf()) {
        return;
    }

    ++depth_;
    currentMenuNode_ = currentMenuNode_->child(index_);
}

void MenuController::back() {
    if (currentMenuNode_ == currentMenuNode_->parent()) {
        LOG("Now on root\r\n");
    } else if (currentMenuNode_ == nullptr) {
        return;
    }

    --depth_;
    currentMenuNode_ = currentMenuNode_->parent();
}

void MenuController::setChild(uint8_t index) {
    currentMenuNode_ = currentMenuNode_->child(index);
    enter();
}

const MenuNode* MenuController::currentMenuNode() const {
    return currentMenuNode_;
}

void MenuController::currentInfo() const {
    printf("=== Selector ===\n");
    printf("Current Menu: %s\n", currentMenuNode_->name());
    printf("Children Count: %d\r\n", currentMenuNode_->childCount());
    printf("Depth: %d\n", depth_);
    printf("================\n");
}

void MenuController::tree() const {
    menu_.tree();
}

uint8_t MenuController::index() const {
    return index_;
}