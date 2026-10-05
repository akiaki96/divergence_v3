# スラロームのターンの種類（slalom_profile_designer.py と gen_slalom_params.py で共有する）
#
# 座標はデザイナーの盤面（クラシック迷路，1区画180mm）：壁は x = -90, 90, 270 / y = 0, 180, 360 の線上。
# 角度はy軸（前方）から時計回り（右旋回が正）。
#   小回り90°: 区画境界(壁の中央)から入り，1区画内で曲がって隣の境界から出る
#   大回り90°: 区画中央から曲がり始め，斜め隣の区画中央で曲がり終える
#
# path は，入口の基準点を (0, 0) とした基準の折れ線 [mm]（右旋回，external/micromouse_simulator の action.json と
# maze_logic.py の半区画の格子：直進1歩が90mm，斜めは (90, 90)）。最後の点が出口の基準点（exit_offset）。
# デザイナーの「出口のずれ」，identify_slip.py，slalom_autotune.py が使う。通り抜ける区画境界には壁がない（slalom_sim.obstacles）
#
# entry / exit は，入口・出口の基準点（オフセットの起点・終点）が区画のどこにあるか：
#   "edge"   … 区画境界（壁の中央）
#   "center" … 区画中央
#   "diag"   … 斜め走行側（実機の試験は未対応）
from dataclasses import dataclass


@dataclass(frozen=True)
class Preset:
    key: str          # デザイナー内部のキー
    label: str        # 表示名（slalom_params.json のキーにもなる）
    ini_x: float
    ini_y: float
    ini_angle: float  # [deg]
    fin_angle: float  # [deg]
    cpp_name: str     # 生成ヘッダの識別子（config::slalom::Turn の列挙子と定数名の接頭辞）
    entry: str
    exit: str
    path: tuple

    @property
    def angle(self) -> float:
        return self.fin_angle - self.ini_angle

    @property
    def exit_offset(self) -> tuple:
        """入口の基準点から出口の基準点までの変位 (dx, dy) [mm]"""
        return self.path[-1]


# 斜めの入口・出口は，壁の中央（区画境界の中点）を通る斜めの線の上にある
PRESET_LIST = [
    Preset("s90",    "小回り90°", 0.0, 180.0,  0.0,  90.0, "S90",    "edge",   "edge",
           ((0, 0), (0, 90), (90, 90))),                           # ACT_TURN_*_MOVE
    Preset("l90",    "大回り90°", 0.0,  90.0,  0.0,  90.0, "L90",    "center", "center",
           ((0, 0), (0, 180), (180, 180))),                        # ACT_S90_*
    Preset("180",    "180°",      0.0,  90.0,  0.0, 180.0, "T180",   "center", "center",
           ((0, 0), (0, 180), (180, 180), (180, 0))),              # ACT_S180_*
    Preset("in45",   "入45°",     0.0,  90.0,  0.0,  45.0, "IN45",   "center", "diag",
           ((0, 0), (0, 90), (90, 180))),                          # ACT_S45_in_*
    Preset("out45",  "出45°",     0.0, 180.0, 45.0,  90.0, "OUT45",  "diag",   "center",
           ((0, 0), (90, 90), (180, 90))),                         # ACT_S45_out_*
    Preset("v90",    "V90°",      0.0, 180.0, 45.0, 135.0, "V90",    "diag",   "diag",
           ((0, 0), (90, 90), (180, 0))),                          # ACT_V90_*
    Preset("in135",  "入135°",    0.0,  90.0,  0.0, 135.0, "IN135",  "center", "diag",
           ((0, 0), (0, 180), (90, 180), (180, 90))),              # ACT_S135_in_*
    Preset("out135", "出135°",    0.0, 180.0, 45.0, 180.0, "OUT135", "diag",   "center",
           ((0, 0), (90, 90), (180, 90), (180, -90))),             # ACT_S135_out_*
]

PRESET_BY_LABEL = {p.label: p for p in PRESET_LIST}


# slalom_params.json / slalom_tuning.json の速度のキー。ファン（吸引）を回して走るパラメータは "_fan" を付けて，
# 同じターン・速度でもファンOFF（"500"）とファンON（"500_fan"）を別々に持つ
FAN_SUFFIX = "_fan"


def make_speed_key(speed: float, fan: bool) -> str:
    """500.0, False -> "500" / 512.5, True -> "512.5_fan" """
    return f"{speed:g}" + (FAN_SUFFIX if fan else "")


def parse_speed_key(key: str) -> tuple[float, bool]:
    """"500_fan" -> (500.0, True)"""
    fan = key.endswith(FAN_SUFFIX)
    return float(key[:-len(FAN_SUFFIX)] if fan else key), fan
