#pragma once

#include <array>
#include <cstddef>
#include <utility>

#include "menu/menuNode.hpp"
#include "config/node_func_maker.hpp"
#include "test/slalom_test.hpp"
#include "app/search.hpp"
#include "app/fast_run.hpp"
#include "app/maze_menu.hpp"
#include "test/axle_check_test.hpp"
#include "test/wall_edge_test.hpp"
#include "test/front_check_test.hpp"

// スラロームの種類・速度とも，メニューの子の数の上限に収まるか（収まらなければビルドを止める）
constexpr bool slalomFitsMenu() {
    for (const auto& turn : config::slalom::TURNS) {
        if (turn.count > config::menu::MAX_CHILDREN) return false;
    }
    return config::slalom::TURNS.size() <= config::menu::MAX_CHILDREN;
}
static_assert(slalomFitsMenu(), "slalom turns or speeds exceed config::menu::MAX_CHILDREN");
static_assert(config::search::PRESETS.size() <= config::menu::MAX_CHILDREN,
              "search presets exceed config::menu::MAX_CHILDREN");
static_assert(config::search::TEST_PRESETS.size() <= config::menu::MAX_CHILDREN,
              "test search presets exceed config::menu::MAX_CHILDREN");
static_assert(config::run::PRESETS.size() <= config::menu::MAX_CHILDREN,
              "run presets exceed config::menu::MAX_CHILDREN");

class Menu {
public:
    Menu();

    const MenuNode* root() {
        return &root_;
    };

    void tree() const {
        root_.tree();
    }

private:
    void buildTree();
    void setFunction();

    // スラロームの試験：Slalom → 向き → 種類（config::slalom::TURNS）→ 速度（config::slalom::ALL）。
    // 速度のノードは生成ヘッダのパラメータごとに1つ作り，種類のノードの子にはALLの範囲[first, first+count)を割り当てる
    template <slalom::TurnDir Dir, std::size_t... I>
    static std::array<MenuNode, sizeof...(I)> slalomSpeedNodes(std::index_sequence<I...>) {
        return {MenuNode(config::slalom::ALL[I].speed_name, nullptr, &slalom_test_onenter<Dir, I>)...};
    }

    // 探索：Run → Search → プリセット（config::search::PRESETS，tools/search_presets.json の順）
    template <std::size_t... I>
    static std::array<MenuNode, sizeof...(I)> searchNodes(std::index_sequence<I...>) {
        return {MenuNode(config::search::PRESETS[I].name, nullptr, &search_onenter<I>)...};
    }

    // 最短走行：Run → Fast → プリセット（config::run::PRESETS，tools/run_presets.json の順）
    template <std::size_t... I>
    static std::array<MenuNode, sizeof...(I)> fastNodes(std::index_sequence<I...>) {
        return {MenuNode(config::run::PRESETS[I].name, nullptr, &fast_onenter<I>)...};
    }

    // 試験用の探索：Test → Search → プリセット（config::search::TEST_PRESETS，"menu": "test" のもの）
    template <std::size_t... I>
    static std::array<MenuNode, sizeof...(I)> testSearchNodes(std::index_sequence<I...>) {
        return {MenuNode(config::search::TEST_PRESETS[I].name, nullptr, &test_search_onenter<I>)...};
    }

    template <std::size_t... K>
    static std::array<MenuNode, sizeof...(K)> slalomTurnNodes(std::index_sequence<K...>) {
        return {MenuNode(config::slalom::TURNS[K].name)...};
    }

    template <std::size_t K, std::size_t N>
    static void linkSlalom(MenuNode& dir, std::array<MenuNode, K>& turns, std::array<MenuNode, N>& speeds) {
        dir.setChildren(pointersOf(turns));
        auto speed_pointers = pointersOf(speeds);
        for (std::size_t k = 0; k < K; ++k) {
            const auto& turn = config::slalom::TURNS[k];
            turns[k].setChildren(speed_pointers.data() + turn.first, turn.count);
        }
    }

    template <std::size_t N>
    static std::array<MenuNode*, N> pointersOf(std::array<MenuNode, N>& nodes) {
        std::array<MenuNode*, N> pointers{};
        for (std::size_t i = 0; i < N; ++i) {
            pointers[i] = &nodes[i];
        }
        return pointers;
    }

    static constexpr auto SLALOM_PARAM_INDICES = std::make_index_sequence<config::slalom::ALL.size()>{};
    static constexpr auto SLALOM_TURN_INDICES = std::make_index_sequence<config::slalom::TURNS.size()>{};

private:
    MenuNode root_{"Root"};

    MenuNode log_dump_{"LogDump"};

    MenuNode run_{"Run"};
        MenuNode search_{"Search"};
            std::array<MenuNode, config::search::PRESETS.size()> search_presets_ =
                searchNodes(std::make_index_sequence<config::search::PRESETS.size()>{});
        MenuNode fast_{"Fast"};   // 最短走行（app/fast_run.hpp）。保存した迷路を使う
            std::array<MenuNode, config::run::PRESETS.size()> fast_presets_ =
                fastNodes(std::make_index_sequence<config::run::PRESETS.size()>{});
        MenuNode maze_{"Maze"};   // 保存した迷路（app/maze_store.hpp）
            MenuNode maze_show_{"Show"};
            MenuNode maze_clear_{"Clear"};

    MenuNode device_{"Device"};
        MenuNode imu_{"IMU"};
            MenuNode imu_acc_{"IMU Accel"};
            MenuNode imu_gyro_{"IMU Gyros"};
        MenuNode encoder_{"Encoder"};
            MenuNode encoder_l_{"Encoder Left"};
            MenuNode encoder_r_{"Encoder Right"};
        MenuNode motor_{"Motor"};
            MenuNode motor_l_{"Motor Left"};
            MenuNode motor_r_{"Motor Right"};
                MenuNode right_050_{"Motor Right 0.5"};
            MenuNode motor_velocity_x_{"Motor velocity X"};
                MenuNode velocity_step_300_{"v step +300"};
                MenuNode velocity_step_600_{"v step +600"};
                MenuNode velocity_step_900_{"v step +900"};
                MenuNode velocity_step_000_{"v step 0"};
            MenuNode plan_profile_{"Plan profile"};
                MenuNode plan_step_velocity_{"step velocity"};
                MenuNode plan_vel2vel_{"straight"};
                MenuNode plan_encoder_check_{"encoder check 50"};
                MenuNode plan_encoder_check_fan_{"encoder check 50 fan0.2"};
                MenuNode plan_fast_2000_{"fast 2000 fan0.2"};
            MenuNode plan_rotation_{"Plan rotation"};
                MenuNode plan_turn_pos430_{"turn +430"};
                MenuNode plan_turn_neg430_{"turn -430"};
            MenuNode rotation_{"Rotation"};
                MenuNode rot_angle_hold_{"angle hold"};
                MenuNode rot_pivot_pos90_{"pivot +90"};
                MenuNode rot_pivot_neg90_{"pivot -90"};
                MenuNode rot_pivot_pos180_{"pivot +180"};
                MenuNode rot_pivot_neg180_{"pivot -180"};


        MenuNode fan_{"Fan"};
            MenuNode fan_run_010_{"fan vsag 0.10"};
            MenuNode fan_run_020_{"fan vsag 0.20"};
            MenuNode fan_run_030_{"fan vsag 0.30"};
            MenuNode fan_run_040_{"fan vsag 0.40"};
            MenuNode fan_bringup_{"fan bringup"};
            MenuNode fan_hold_010_{"fan hold 0.10"};
            MenuNode fan_hold_020_{"fan hold 0.20"};
            MenuNode fan_hold_030_{"fan hold 0.30"};
            MenuNode fan_hold_040_{"fan hold 0.40"};
        MenuNode ir_{"IR"};
            MenuNode ir_r_{"IR Right"};
            MenuNode ir_fr_{"IR Front Right"};
            MenuNode ir_fl_{"IR Front Left"};
            MenuNode ir_l_{"IR Left"};
            MenuNode ir_wall_check_{"Wall check"};
            MenuNode ir_front_check_{"Front check"};   // 前左・前右の前壁の閾値（test/front_check_test.hpp）
                MenuNode front_check_wall_{"wall", nullptr, &front_check_onenter<FrontCase::wall>};
                MenuNode front_check_no_wall_{"no wall", nullptr, &front_check_onenter<FrontCase::no_wall>};
                MenuNode front_check_show_{"show", nullptr, &showFrontCheck};
                MenuNode front_check_reset_{"reset", nullptr, &resetFrontCheck};
            MenuNode ir_wall_edge_{"Wall edge"};   // 壁切れの補正の試験（test/wall_edge_test.hpp）
                MenuNode wall_edge_calib_300_{"calib 300", nullptr, &wall_edge_test_onenter<WallEdgeMode::calib, 300>};
                MenuNode wall_edge_calib_500_{"calib 500", nullptr, &wall_edge_test_onenter<WallEdgeMode::calib, 500>};
                MenuNode wall_edge_calib_700_{"calib 700", nullptr, &wall_edge_test_onenter<WallEdgeMode::calib, 700>};
                MenuNode wall_edge_verify_500_{"verify 500", nullptr, &wall_edge_test_onenter<WallEdgeMode::verify, 500>};
                MenuNode wall_edge_inject_500_{"inject 500", nullptr, &wall_edge_test_onenter<WallEdgeMode::inject, 500>};
        MenuNode battery_{"Battery"};
        MenuNode led_{"LED"};

    MenuNode slalom_{"Slalom"};
        MenuNode slalom_left_{"Slalom left"};
            std::array<MenuNode, config::slalom::TURNS.size()> slalom_left_turns_ = slalomTurnNodes(SLALOM_TURN_INDICES);
                std::array<MenuNode, config::slalom::ALL.size()> slalom_left_speeds_ = slalomSpeedNodes<slalom::TurnDir::left>(SLALOM_PARAM_INDICES);
        MenuNode slalom_right_{"Slalom right"};
            std::array<MenuNode, config::slalom::TURNS.size()> slalom_right_turns_ = slalomTurnNodes(SLALOM_TURN_INDICES);
                std::array<MenuNode, config::slalom::ALL.size()> slalom_right_speeds_ = slalomSpeedNodes<slalom::TurnDir::right>(SLALOM_PARAM_INDICES);
        MenuNode axle_check_{"Axle check"};   // BACK_TO_AXLE_MMの確認（test/axle_check_test.hpp）
            MenuNode axle_check_n1_{"n=1", nullptr, &axle_check_onenter<1>};
            MenuNode axle_check_n2_{"n=2", nullptr, &axle_check_onenter<2>};
            MenuNode axle_check_n4_{"n=4", nullptr, &axle_check_onenter<4>};
            MenuNode axle_check_n8_{"n=8", nullptr, &axle_check_onenter<8>};

    MenuNode test_{"Test"};
        MenuNode test_search_{"Search"};   // 試験用の探索（近いゴールで往復する等）
            std::array<MenuNode, config::search::TEST_PRESETS.size()> test_search_presets_ =
                testSearchNodes(std::make_index_sequence<config::search::TEST_PRESETS.size()>{});

};
