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
        return (index < childCount_) ? childPool_[childFirst_ + index] : nullptr;
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

    void setOnSelected(MenuAction onSelected) {
        onSelected_ = onSelected;
    }

    void setOnEnter(MenuAction onEnter) {
        onEnter_ = onEnter;
    }

    void tree(uint8_t indent = 0) const;
    void setParentRec();
    void setParent(MenuNode* parent);

    template<std::size_t N>
    void setChildren(const std::array<MenuNode*, N>& children) {
        static_assert(N <= config::menu::MAX_CHILDREN);
        setChildren(children.data(), N);
    }

    // children[0]からcount個を子にする（個数がノードごとに違う，生成した表から並べるとき用）。
    // 上限の検査は呼び出し側でstatic_assertすること（ここではMAX_CHILDRENで切り詰める）。
    // 子へのポインタは全ノードで共有する表（childPool_）の続きに写す。メニューを組み立てるとき（起動時）に
    // ノードごとに1回だけ呼ぶこと（呼び直すと表を余分に使う）
    void setChildren(MenuNode* const* children, std::size_t count);

private:
    // 子へのポインタの共有の表。ノードの大半は葉なので，ノードごとに MAX_CHILDREN 個の配列を持つより RAM が少ない
    static MenuNode* childPool_[config::menu::MAX_CHILD_LINKS];
    static uint16_t childPoolUsed_;

    const char* name_;
    MenuNode* parent_ = nullptr;

    MenuAction onSelected_ = nullptr;
    MenuAction onEnter_ = nullptr;

    uint16_t childFirst_ = 0;   // 子は childPool_[childFirst_] から childCount_ 個
    uint8_t childCount_ = 0;
};