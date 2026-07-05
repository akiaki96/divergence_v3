#include "menu/menuNode.hpp"
#include "common/debug.hpp"

MenuNode::MenuNode(const char* name)
    : name_(name)
{}

MenuNode::MenuNode(const char* name, MenuAction onSelected, MenuAction onEnter)
    : name_(name),
    onSelected_(onSelected),
    onEnter_(onEnter)
{}

void MenuNode::tree(uint8_t indent) const {
    for (uint8_t i = 0; i < indent; ++i) {
        LOG("  ");
    }
    LOG("-%s\r\n", name_);
    for (uint8_t i = 0; i < childCount_; ++i) {
        children_[i]->tree(indent + 1);
    }
}

void MenuNode::setParentRec() {
    if (isLeaf()) {
        return;
    }
    for (uint8_t i = 0; i < childCount_; ++i) {
        children_[i]->parent_ = this;
        children_[i]->setParentRec();
    }
}

void MenuNode::setParent(MenuNode* parent) {
    parent_ = parent;
}