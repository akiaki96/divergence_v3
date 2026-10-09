#include "menu/menuNode.hpp"
#include "common/debug.hpp"

MenuNode* MenuNode::childPool_[config::menu::MAX_CHILD_LINKS] = {};
uint16_t MenuNode::childPoolUsed_ = 0;

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
    LOG("- %s\r\n", name_);
    for (uint8_t i = 0; i < childCount_; ++i) {
        childPool_[childFirst_ + i]->tree(indent + 1);
    }
}

void MenuNode::setParentRec() {
    if (isLeaf()) {
        return;
    }
    for (uint8_t i = 0; i < childCount_; ++i) {
        MenuNode* c = childPool_[childFirst_ + i];
        c->parent_ = this;
        c->setParentRec();
    }
}

void MenuNode::setParent(MenuNode* parent) {
    parent_ = parent;
}
void MenuNode::setChildren(MenuNode* const* children, std::size_t count) {
    if (count > config::menu::MAX_CHILDREN) {
        count = config::menu::MAX_CHILDREN;
    }
    std::size_t room = config::menu::MAX_CHILD_LINKS - childPoolUsed_;
    if (count > room) {
        LOG("menu: child pool full (%s: %u of %u children fit, raise config::menu::MAX_CHILD_LINKS)\r\n",
            name_, static_cast<unsigned>(room), static_cast<unsigned>(count));
        count = room;
    }
    childFirst_ = childPoolUsed_;
    childCount_ = static_cast<uint8_t>(count);
    for (std::size_t i = 0; i < count; ++i) {
        childPool_[childFirst_ + i] = children[i];
    }
    childPoolUsed_ = static_cast<uint16_t>(childPoolUsed_ + count);
}
