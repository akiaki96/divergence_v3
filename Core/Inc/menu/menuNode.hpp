#pragma once

#include <cstdint>
#include <array>

#include "config/mouse_config.hpp"

using MenuAction = void (*)();

class MenuNode {
public:
    MenuNode(const char* name);
    MenuNode(const char* name, MenuAction onSelected, MenuAction onEnter);

    const char* name() const {
        return name_;
    }

    const MenuNode* parent() const {
        return parent_;
    }

    const MenuNode* child(uint8_t index) const {
        return (index < childCount_) ? children_[index] : nullptr;
    }

    const uint8_t childCount() const {
        return childCount_ ;
    }

    const bool isLeaf() const {
        return (childCount_ == 0);
    }

    MenuAction onSelected() const {
        return onSelected_;
    }

    MenuAction onEnter() const {
        return onEnter_;
    }

    void tree(uint8_t indent = 0) const;
    void setParentRec();

    template<std::size_t N>
    void setChildren(const std::array<MenuNode*, N>& children) {
        static_assert(N <= config::menu::MAX_CHILDREN);
        childCount_ = N;
        for (std::size_t i = 0; i < N; ++i) {
            children_[i] = children[i];
        }
    }

private:
    const char* name_;
    MenuNode* parent_ = nullptr;

    std::array<MenuNode*, config::menu::MAX_CHILDREN> children_{};
    uint8_t childCount_ = 0;

    MenuAction onSelected_ = nullptr;
    MenuAction onEnter_ = nullptr;
};