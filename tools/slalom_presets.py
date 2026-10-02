# スラロームのターンの種類（slalom_profile_designer.py と gen_slalom_params.py で共有する）
#
# 座標はデザイナーの盤面（クラシック迷路，1区画180mm）：壁は x = -90, 90, 270 / y = 0, 180, 360 の線上。
# 角度はy軸（前方）から時計回り（右旋回が正）。
#   小回り90°: 区画境界(壁の中央)から入り，1区画内で曲がって隣の境界から出る
#   大回り90°: 区画中央から曲がり始め，斜め隣の区画中央で曲がり終える
#
# exit_offset は，入口の基準点から出口の基準点までの変位 (dx, dy) [mm]（右旋回，external/micromouse_simulator の
# action.json の折れ線と同じ）。デザイナーの「出口のずれ」と identify_slip.py が使う。斜めのターンは未設定（None）
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
    exit_offset: tuple | None = None

    @property
    def angle(self) -> float:
        return self.fin_angle - self.ini_angle


# 斜めのターンの直線側（in45/in135の入口，out45/out135の出口）は，試験が斜め側で未対応になるため未使用（要確認）
PRESET_LIST = [
    Preset("s90",    "小回り90°", 0.0, 180.0,  0.0,  90.0, "S90",    "edge",   "edge",   (90.0, 90.0)),     # ACT_TURN_*_MOVE
    Preset("l90",    "大回り90°", 0.0,  90.0,  0.0,  90.0, "L90",    "center", "center", (180.0, 180.0)),   # ACT_S90_*
    Preset("180",    "180°",      0.0,  90.0,  0.0, 180.0, "T180",   "center", "center", (180.0, 0.0)),     # ACT_S180_*
    Preset("in45",   "入45°",     0.0,  90.0,  0.0,  45.0, "IN45",   "center", "diag"),
    Preset("out45",  "出45°",     0.0, 180.0, 45.0,  90.0, "OUT45",  "diag",   "center"),
    Preset("v90",    "V90°",      0.0, 180.0, 45.0, 135.0, "V90",    "diag",   "diag"),
    Preset("in135",  "入135°",    0.0,  90.0,  0.0, 135.0, "IN135",  "center", "diag"),
    Preset("out135", "出135°",    0.0, 180.0, 45.0, 180.0, "OUT135", "diag",   "center"),
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
