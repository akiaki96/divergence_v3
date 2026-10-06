# slalom_profile_designer.py  （レイアウト修正版）
import pygame
import sys
import numpy as np
from functools import partial
import math
import os
import json
import datetime
import pyperclip

from slalom_presets import PRESET_LIST, make_speed_key, parse_slalom_key, slalom_key_label, slalom_key_order
import slalom_sim

PRESET_BY_KEY = {p.key: p for p in PRESET_LIST}

pygame.init()
FONT_PATH = "/Users/inabeshuuyou/Library/Fonts/YujiSyuku-Regular.ttf"
FONT = pygame.font.Font(FONT_PATH, 15)
BIGFONT = pygame.font.Font(FONT_PATH, 20)

# --- world coordinate extents (match roughly the original matplotlib axes) ---
WORLD_MIN_X = -100.0
WORLD_MAX_X = 280.0
WORLD_MIN_Y = -10.0
WORLD_MAX_Y = 370.0

# --- screen layout ---
SCREEN_W = 1100
SCREEN_H = 760
CANVAS_SIZE = 540  # square drawing area
CANVAS_LEFT = 200
CANVAS_TOP = 40

# Colors
BG = (245, 245, 245)
PANEL = (230, 230, 230)
GRID_COLOR = (170, 170, 170)
PILLAR_COLOR = (200, 30, 30)
WALL_COLOR = (235, 170, 170)   # ありうる壁（区画境界のうち，基準の折れ線が通り抜けないもの）
LINE_BLUE = (5, 5, 255)
LINE_GREEN = (40, 255, 40)
LINE_RED = (255, 40, 40)
GREY_LINE = (136, 136, 136)
TEXT_COLOR = (10, 10, 10)
BUTTON_COLOR = (200, 200, 200)
BUTTON_HOVER = (180, 180, 180)
INPUT_BG = (255, 255, 255)

screen = pygame.display.set_mode((SCREEN_W, SCREEN_H))
pygame.display.set_caption("たーんしみゅれーた (pygame)")

# スクロールで 1 ステップ進むのに必要なスクロール量（トラックパッドで速すぎる場合は大きくする）
SCROLL_UNIT = 1.0
HOVER_BG = (238, 238, 250)

class NumericInput:
    def __init__(self, x, y, w, h, initial_value=0.0, step=1.0):
        self.rect = pygame.Rect(x, y, w, h)
        self.value = initial_value
        self.step = step
        self.text = str(initial_value)

        # + / - ボタン (正方形に近い)
        btn_w = h
        self.minus_rect = pygame.Rect(x - btn_w - 6, y, btn_w, h)
        self.plus_rect = pygame.Rect(x + w + 6, y, btn_w, h)

        self.active = False
        self.font = FONT  # use same FONT for visual consistency

        # スクロール操作を受け付ける範囲（既定は −ボタン〜＋ボタン。ラベル込みの行に広げて使う）
        self.scroll_rect = self.minus_rect.union(self.plus_rect)
        self.scroll_acc = 0.0  # トラックパッドの細かいスクロール量を貯める

    def _nudge(self, n):
        """step を n 回分増減して軌道を再生成する"""
        self.value = round(self.get_value() + n * self.step, 9)  # 0.001 刻みの誤差蓄積を防ぐ
        self.text = self._format_value(self.value)
        on_generate()

    def _handle_wheel(self, event):
        if not self.scroll_rect.collidepoint(pygame.mouse.get_pos()):
            self.scroll_acc = 0.0
            return
        dy = getattr(event, "precise_y", event.y)
        if getattr(event, "flipped", False):
            dy = -dy  # macOS のナチュラルスクロールを打ち消し「上へスクロール = 増加」にそろえる
        if dy == 0:
            return
        if self.scroll_acc * dy < 0:
            self.scroll_acc = 0.0  # 向きが変わったら貯めた分を捨てて即反応させる
        self.scroll_acc += dy
        notches = int(self.scroll_acc / SCROLL_UNIT)  # 0 方向へ切り捨て
        if notches == 0:
            return
        self.scroll_acc -= notches * SCROLL_UNIT

        mods = pygame.key.get_mods()
        scale = 1.0
        if mods & pygame.KMOD_ALT:
            scale = 0.1   # ⌥: 微調整
        elif mods & (pygame.KMOD_META | pygame.KMOD_CTRL):
            scale = 10.0  # ⌘ / Ctrl: 粗調整
        self._nudge(notches * scale)

    def handle_event(self, event):
        if event.type == pygame.MOUSEWHEEL:
            self._handle_wheel(event)
            return

        # マウス処理
        if event.type == pygame.MOUSEBUTTONDOWN and event.button == 1:
            if self.rect.collidepoint(event.pos):
                self.active = True
            else:
                # クリックが入力欄以外なら非アクティブ化
                if not (self.plus_rect.collidepoint(event.pos) or self.minus_rect.collidepoint(event.pos)):
                    self.active = False

            # − ボタン
            if self.minus_rect.collidepoint(event.pos):
                self._nudge(-1)

            # ＋ ボタン
            if self.plus_rect.collidepoint(event.pos):
                self._nudge(+1)

        # キーボード入力（アクティブ時）
        if event.type == pygame.KEYDOWN and self.active:
            if event.key == pygame.K_RETURN or event.key == pygame.K_KP_ENTER:
                # Enter で編集終了（値を固定）
                try:
                    self.value = float(self.text)
                except:
                    pass
                self.active = False
            elif event.key == pygame.K_BACKSPACE:
                self.text = self.text[:-1]
            elif event.unicode and (event.unicode.isdigit() or event.unicode in ['.', '-']):
                self.text += event.unicode
            elif event.key == pygame.K_UP:
                self._nudge(+1)
            elif event.key == pygame.K_DOWN:
                self._nudge(-1)

            # 可能なら文字列を数値に変換（エラーは無視）
            try:
                self.value = float(self.text)
            except:
                pass

    def draw(self, surface):
        # マウスが乗っている行を薄く塗り、スクロール対象を分かるようにする
        hovered = self.scroll_rect.collidepoint(pygame.mouse.get_pos())
        if hovered:
            pygame.draw.rect(surface, HOVER_BG, self.scroll_rect, border_radius=4)

        # 入力欄
        pygame.draw.rect(surface, INPUT_BG, self.rect)
        # border highlight if active / hovered
        border_col = (120, 120, 220) if (self.active or hovered) else (150,150,150)
        pygame.draw.rect(surface, border_col, self.rect, 2)

        # テキスト (右寄せで見やすく)
        txt_img = self.font.render(self.text, True, TEXT_COLOR)
        # right align inside rect with some padding
        tx = self.rect.x + max(4, self.rect.w - txt_img.get_width() - 6)
        ty = self.rect.y + (self.rect.h - txt_img.get_height()) // 2
        surface.blit(txt_img, (tx, ty))

        # − ボタン
        pygame.draw.rect(surface, (230, 230, 230), self.minus_rect)
        pygame.draw.rect(surface, (120,120,120), self.minus_rect, 2)
        minus_img = self.font.render("-", True, TEXT_COLOR)
        surface.blit(minus_img, (self.minus_rect.centerx - minus_img.get_width()//2, self.minus_rect.centery - minus_img.get_height()//2))

        # ＋ ボタン
        pygame.draw.rect(surface, (230, 230, 230), self.plus_rect)
        pygame.draw.rect(surface, (120,120,120), self.plus_rect, 2)
        plus_img = self.font.render("+", True, TEXT_COLOR)
        surface.blit(plus_img, (self.plus_rect.centerx - plus_img.get_width()//2, self.plus_rect.centery - plus_img.get_height()//2))

    def set_value(self, v):
        self.value = float(v)
        self.text = self._format_value(self.value)

    def get_value(self):
        # 最新の text を value に反映して返す
        try:
            self.value = float(self.text)
        except:
            pass
        return self.value

    def _format_value(self, v):
        # 整数に見える場合は整数表記、それ以外は末尾の 0 を省いた小数（⌥ の微調整で 4 桁以上にもなる）
        if abs(v - round(v)) < 1e-9:
            return str(int(round(v)))
        else:
            return f"{v:.6f}".rstrip("0")

# --- utility coordinate transform: world (mm) -> canvas pixels ---
def world_to_screen(x, y):
    """
    Map world coordinates to canvas pixel coordinates.
    world y increases upward; screen y increases downward.
    """
    sx = CANVAS_LEFT + (x - WORLD_MIN_X) / (WORLD_MAX_X - WORLD_MIN_X) * CANVAS_SIZE
    sy = CANVAS_TOP + CANVAS_SIZE - (y - WORLD_MIN_Y) / (WORLD_MAX_Y - WORLD_MIN_Y) * CANVAS_SIZE
    return int(sx), int(sy)

# --- UI widgets (very small, self-contained) ---
class Button:
    def __init__(self, rect, text, callback):
        self.rect = pygame.Rect(rect)
        self.text = text
        self.callback = callback
    def draw(self, surf):
        mx, my = pygame.mouse.get_pos()
        color = BUTTON_HOVER if self.rect.collidepoint((mx,my)) else BUTTON_COLOR
        pygame.draw.rect(surf, color, self.rect, border_radius=6)
        txt = FONT.render(self.text, True, TEXT_COLOR)
        surf.blit(txt, txt.get_rect(center=self.rect.center))
    def handle_event(self, ev):
        if ev.type == pygame.MOUSEBUTTONDOWN and ev.button == 1:
            if self.rect.collidepoint(ev.pos):
                self.callback()

# (旧 TextBox クラスは残しておくが、右パネルの入力は NumericInput に統一)
class TextBox:
    def __init__(self, rect, text="", allowed=None, max_len=32):
        self.rect = pygame.Rect(rect)
        self.text = text
        self.allowed = allowed   # 入力できる文字（None なら何でも）
        self.max_len = max_len
        self.active = False
        self.cursor_visible = True
        self.cursor_timer = 0.0
    def draw(self, surf):
        pygame.draw.rect(surf, INPUT_BG, self.rect)
        pygame.draw.rect(surf, (150,150,150), self.rect, 1)
        txt = FONT.render(self.text, True, TEXT_COLOR)
        surf.blit(txt, (self.rect.x+4, self.rect.y+4))
        # cursor blink
        if self.active:
            self.cursor_timer += clock.get_time()/1000.0
            if self.cursor_timer > 0.5:
                self.cursor_visible = not self.cursor_visible
                self.cursor_timer = 0.0
            if self.cursor_visible:
                cx = self.rect.x+4 + txt.get_width()+1
                pygame.draw.line(surf, TEXT_COLOR, (cx, self.rect.y+4), (cx, self.rect.y+4 + txt.get_height()))
    def handle_event(self, ev):
        if ev.type == pygame.MOUSEBUTTONDOWN and ev.button == 1:
            self.active = self.rect.collidepoint(ev.pos)
        if self.active and ev.type == pygame.KEYDOWN:
            if ev.key == pygame.K_RETURN:
                self.active = False
            elif ev.key == pygame.K_BACKSPACE:
                self.text = self.text[:-1]
            elif ev.key == pygame.K_v and (pygame.key.get_mods() & pygame.KMOD_CTRL):
                # paste
                try:
                    import pyperclip
                    pasted = pyperclip.paste()
                    if self.allowed is not None:
                        pasted = "".join(c for c in pasted if c in self.allowed)
                    self.text = (self.text + pasted)[:self.max_len]
                except:
                    pass
            else:
                char = ev.unicode
                if char and (self.allowed is None or char in self.allowed) and len(self.text) < self.max_len:
                    self.text += char

# --- simulation global defaults (set_angle_preset で上書きされる) ---
ini_x = 0.0
ini_y = 180.0
ini_angle = 0.0
fin_angle = 90.0

# Create UI elements
buttons = []
# textboxes dict kept for backward compatibility but we won't draw them in right panel
textboxes = {}

# parameter labels and defaults (same names as tkinter fields)
# Keep params list for labels; keys correspond to inputs dict keys
params = [
    ("速度(mm/s)", "300", "Set_Speed"),
    ("最大角速度 °/s", "200", "Set_low_AngVel"),
    ("角加速度", "1000", "Set_Low_AngAcl"),
    ("入口オフセット(mm)", "90", "Set_pri_offset"),
    ("出口オフセット(mm)", "55", "Set_post_offset"),
    ("滑り係数 K", "0.002", "Set_K_SP"),
    ("滑り係数 c(mm)", "0", "Set_C_SP"),
    ("機体の横幅(mm)", "86", "Set_Width")
]
# 軌跡と滑り角 β = K·v·ω + c·ω/v のモデルは slalom_sim.py（slalom_autotune.py・identify_slip.py と共有）。
# K・c はどちらも機体・床・ファンの有無で変わる。片方を0にすればもう片方だけのモデルになる
# 保存データに無い項目を読み込むときの値（以前の保存データとの互換）
LOAD_DEFAULTS = {"Set_K_SP": 0.0, "Set_C_SP": 0.0}

# ターンプリセット: key -> (表示名, ini_x, ini_y, ini_angle, fin_angle)
# 定義は slalom_presets.py（生成スクリプト gen_slalom_params.py と共有）
PRESETS = {p.key: (p.label, p.ini_x, p.ini_y, p.ini_angle, p.fin_angle) for p in PRESET_LIST}

# 現在選択中のプリセット（保存時のキーに使う）
current_preset = "s90"

# angle preset callbacks
def set_angle_preset(key):
    global ini_x, ini_y, ini_angle, fin_angle, current_preset
    current_preset = key
    _, ini_x, ini_y, ini_angle, fin_angle = PRESETS[key]
    # 同じ速度で保存済みのパラメーターがあれば読み込む
    load_params(silent_if_missing=True)

# create preset buttons on left
preset_buttons = {}
for i, key in enumerate(PRESETS):
    b = Button((24, 40 + i*46, 140, 36), PRESETS[key][0], partial(set_angle_preset, key))
    buttons.append(b)
    preset_buttons[key] = b

# --- パラメーター保存 / 読込 ---
# 構造: { "小回り90°": { "500": { "Set_Speed": 500, ..., "result": {...}, "saved_at": "..." } } }
# キーは "<速度>[_fan][_<組>]"（slalom_presets.py）。同じ速度・ファンで別の設計を試すときは「組」に名前を入れて保存する
SAVE_PATH = os.path.join(os.path.dirname(os.path.abspath(__file__)), "slalom_params.json")
status_msg = ""
last_result = {}
saved_data = {}  # 保存ファイルの内容のキャッシュ（描画で毎フレーム参照するため）

def preset_name(key):
    return PRESETS[key][0]

def speed_key(speed):
    # 500.0 -> "500"（ファンOFF）/ "500_fan"（ファンON）/ "500_b"（組 b）。ファンのON/OFF・組は別々に保存する。
    # 組の名前が使えなければ ValueError
    return make_speed_key(speed, fan_on, variant_box.text)

def speed_label(key):
    """保存済みの一覧用：500 / 500 fan / 500 b"""
    return slalom_key_label(key)

def entry_label(name, key):
    """メッセージ用：大回り90° / 500mm/s ファンON / 組 b"""
    speed, fan, variant = parse_slalom_key(key)
    return f"{name} / {speed:g}mm/s ファン{'ON' if fan else 'OFF'}" + (f" / 組 {variant}" if variant else "")

def read_save_file():
    if not os.path.exists(SAVE_PATH):
        return {}
    try:
        with open(SAVE_PATH, "r", encoding="utf-8") as f:
            return json.load(f)
    except (OSError, json.JSONDecodeError) as e:
        print(f"保存ファイルの読込に失敗: {e}")
        return {}

def write_save_file(data):
    # 書き込み途中で落ちてもファイルが壊れないよう一時ファイル経由で置き換える
    tmp_path = SAVE_PATH + ".tmp"
    with open(tmp_path, "w", encoding="utf-8") as f:
        json.dump(data, f, ensure_ascii=False, indent=2)
    os.replace(tmp_path, SAVE_PATH)

# ファン（吸引）を回して走る条件か。滑り係数 K・c はファンの有無で変わるので，パラメータと一緒に保存する
fan_on = False

def toggle_fan():
    global fan_on
    fan_on = not fan_on
    fan_button.text = fan_label()
    # ファンON/OFFは別々に保存しているので，切り替えた側の保存データがあれば読み込む（無ければ今の値のまま）
    load_params(silent_if_missing=True)

def fan_label():
    return "ファン: ON" if fan_on else "ファン: OFF"

def save_params():
    global status_msg, saved_data
    on_generate()  # 保存する結果を現在の入力値と一致させる
    entry = {key: inputs[key].get_value() for _, _, key in params}
    entry["fan"] = fan_on
    entry["result"] = dict(last_result)
    entry["saved_at"] = datetime.datetime.now().isoformat(timespec="seconds")

    name = preset_name(current_preset)
    try:
        spd = speed_key(entry["Set_Speed"])
    except ValueError as e:
        status_msg = f"保存失敗: {e}"
        return
    data = read_save_file()  # 他で編集された分を消さないよう保存直前に読み直す
    data.setdefault(name, {})[spd] = entry
    try:
        write_save_file(data)
        saved_data = data
        status_msg = f"保存: {entry_label(name, spd)}"
    except OSError as e:
        status_msg = f"保存失敗: {e}"

def load_params(silent_if_missing=False):
    global status_msg, saved_data, fan_on
    saved_data = read_save_file()
    name = preset_name(current_preset)
    try:
        spd = speed_key(inputs["Set_Speed"].get_value())
    except ValueError as e:
        status_msg = f"読込失敗: {e}"
        return
    entry = saved_data.get(name, {}).get(spd)
    if entry is None:
        status_msg = "" if silent_if_missing else f"保存データなし: {entry_label(name, spd)}"
        on_generate()
        return
    for _, _, key in params:
        if key in entry:
            inputs[key].set_value(entry[key])
        elif key in LOAD_DEFAULTS:
            inputs[key].set_value(LOAD_DEFAULTS[key])   # 以前の保存データに無い項目は既定値に戻す
    fan_on = parse_slalom_key(spd).fan   # キーが決める（"_fan" ならON）
    fan_button.text = fan_label()
    status_msg = f"読込: {entry_label(name, spd)}"
    on_generate()

def saved_speeds_for_current():
    entries = saved_data.get(preset_name(current_preset), {})
    return [speed_label(k) for k in sorted(entries.keys(), key=slalom_key_order)]

# DrawTrace will be called to render the trace onto a surface
def parse_float(s, fallback=0.0):
    try:
        return float(s)
    except:
        return fallback

def draw_canvas(surface):
    """Draw grid, slanted lines and pillars (approximate original)"""
    # background of canvas
    pygame.draw.rect(surface, (255,255,255), (CANVAS_LEFT, CANVAS_TOP, CANVAS_SIZE, CANVAS_SIZE))
    # grid lines: verticals at x = -90,0,90,180,270 ; horizontals at 0,90,180,270,360
    verticals = [-90.0, 0.0, 90.0, 180.0, 270.0]
    horizontals = [0.0, 90.0, 180.0, 270.0, 360.0]
    for x in verticals:
        p1 = world_to_screen(x, 0.0)
        p2 = world_to_screen(x, 360.0)
        pygame.draw.line(surface, GRID_COLOR, p1, p2, 1)
    for y in horizontals:
        p1 = world_to_screen(-90.0, y)
        p2 = world_to_screen(270.0, y)
        pygame.draw.line(surface, GRID_COLOR, p1, p2, 1)
    # slanted lines (right-up)
    segs = [
        ((0.0,360.0),(-90.0,270.0)),
        ((180.0,360.0),(-90.0,90.0)),
        ((270.0,270.0),(0.0,0.0)),
        ((270.0,90.0),(180.0,0.0)),
        # right-down
        ((180.0,360.0),(270.0,270.0)),
        ((0.0,360.0),(270.0,90.0)),
        ((-90.0,270.0),(180.0,0.0)),
        ((-90.0,90.0),(0.0,0.0)),
    ]
    for (ax,ay),(bx,by) in segs:
        pygame.draw.line(surface, GRID_COLOR, world_to_screen(ax,ay), world_to_screen(bx,by), 1)
    # 柱（12mm角）と，このターンで（探索中に）ありうる壁。位置は slalom_sim と同じ（余裕の計算に使うもの）
    for cx, cy, hx, hy in slalom_sim.obstacles(PRESET_BY_KEY[current_preset]):
        p_tl = world_to_screen(cx - hx, cy + hy)  # note y flipped
        p_br = world_to_screen(cx + hx, cy - hy)
        rect = pygame.Rect(p_tl, (p_br[0]-p_tl[0], p_br[1]-p_tl[1]))
        color = PILLAR_COLOR if hx == hy else WALL_COLOR
        pygame.draw.rect(surface, color, rect)

# --- 追加: グローバル変数で軌道を保持 ---
trace_segments = []
# ここに表示用情報行を格納する（on_generate が書き換える）
info_lines = []

def draw_trace(surface):
    """trace_segments に保存された軌道を描画"""
    for (x1, y1), (x2, y2), color in trace_segments:
        p1 = world_to_screen(x1, y1)
        p2 = world_to_screen(x2, y2)
        if color == GREY_LINE:
            pygame.draw.line(surface, color, p1, p2, 2)
        else:
            pygame.draw.line(surface, color, p1, p2, 2)

def exit_error_lines(traj, speed, omega, alpha, pre, post, c_mm, k):
    """出口のずれ：出口オフセットの終点と出口の基準点（action の折れ線の出口）の差を，出口の向きに対して
    外側（入口と反対側が正）と前後（先が正）に分けて出す。実機の試験で止まった位置を測るときと同じ向き。
    出口の位置はオフセットに対して1次なので，ずれを0にする pre/post も厳密に解ける（slalom_sim.solve_offsets）"""
    p = PRESET_BY_KEY[current_preset]
    outward, longitudinal = slalom_sim.exit_error(p, traj.end_x, traj.end_y)
    lines = [f"出口のずれ: 外側 {outward:+.1f} / 前後 {longitudinal:+.1f} mm，向き {traj.end_angle - fin_angle:+.2f}°"]
    if slalom_sim.parallel(p):
        _, q, lateral = slalom_sim.solve_offsets(p, speed, omega, alpha, c_mm, k, pre=pre)
        lines.append(f"→ 出口オフセット {q:.1f}（横 {lateral:+.1f} は ω・α で直す）")
    else:
        pp, q, _ = slalom_sim.solve_offsets(p, speed, omega, alpha, c_mm, k)
        lines.append(f"→ 入口 {pp:.1f} / 出口 {q:.1f} で 0")
    return lines

# 軌跡の色（フェーズごと）
PHASE_COLOR = {slalom_sim.PRE: LINE_RED, slalom_sim.ACCEL: LINE_BLUE, slalom_sim.CRUISE: LINE_GREEN,
               slalom_sim.DECEL: LINE_BLUE, slalom_sim.POST: LINE_RED}

# --- on_generate uses inputs dict for all parameters ---
def on_generate(cp = False):
    """入力欄の値で軌跡を計算して（slalom_sim.simulate），描く線と表示を作り直す"""
    global info_lines, trace_segments, last_result

    speed = inputs["Set_Speed"].get_value()           # [mm/s]
    low_AngVel = inputs["Set_low_AngVel"].get_value() # [deg/s]
    Low_AngAcl = inputs["Set_Low_AngAcl"].get_value() # [deg/s^2]
    pri_offset = inputs["Set_pri_offset"].get_value() # [mm]
    post_offset = inputs["Set_post_offset"].get_value()# [mm]
    K_slip_angle = inputs["Set_K_SP"].get_value()     # [coef]
    c_slip_mm = inputs["Set_C_SP"].get_value()        # [mm]
    Width = inputs["Set_Width"].get_value()           # [mm]
    if speed <= 0.0 or low_AngVel <= 0.0 or Low_AngAcl <= 0.0:
        info_lines = ["速度・最大角速度・角加速度は正にする"]
        trace_segments = []
        return

    preset = PRESET_BY_KEY[current_preset]
    t = slalom_sim.simulate(preset, speed, low_AngVel, Low_AngAcl, pri_offset, post_offset, c_slip_mm, K_slip_angle)

    # 重心の軌跡（フェーズで色分け）と，機体の横幅の両端の軌跡（灰色）
    lines = []
    h = np.deg2rad(t.headings)
    side_x, side_y = (Width / 2) * np.cos(h), -(Width / 2) * np.sin(h)
    for i in range(1, len(t.xs)):
        lines.append(((t.xs[i-1], t.ys[i-1]), (t.xs[i], t.ys[i]), PHASE_COLOR[t.phases[i]]))
        for sgn in (1.0, -1.0):
            lines.append(((t.xs[i-1] + sgn * side_x[i], t.ys[i-1] + sgn * side_y[i]),
                          (t.xs[i] + sgn * side_x[i], t.ys[i] + sgn * side_y[i]), GREY_LINE))

    info_lines = [
        f"acc   : {t.acc_dist} mm",
        f"const : {t.const_dist} mm",
        f"total : {t.total_dist} mm",
        f"time  : {t.time_ms} ms",
        f"slip  : β max {float(slalom_sim.slip_deg(low_AngVel, speed, c_slip_mm, K_slip_angle)):.2f}°（fan {'ON' if fan_on else 'OFF'}）",
        f"余裕  : 柱・壁まで {slalom_sim.clearance(preset, t, Width):.1f} mm",
    ] + exit_error_lines(t, speed, low_AngVel, Low_AngAcl, pri_offset, post_offset, c_slip_mm, K_slip_angle)

    last_result = {
        "acc_dist": t.acc_dist,
        "const_dist": t.const_dist,
        "total_dist": t.total_dist,  # 経路長（入口オフセット〜出口オフセットまでの重心軌跡）
        "time_ms": t.time_ms,
    }

    if cp:
        pyperclip.copy(f"{speed}, {pri_offset}, {Low_AngAcl}, {t.acc_dist}, {low_AngVel}, {t.const_dist}, {post_offset}")
    trace_segments = lines

# generate button (unchanged position)
gen_button = Button((CANVAS_LEFT + CANVAS_SIZE - 120, CANVAS_TOP + CANVAS_SIZE + 8, 120, 34), "軌道生成", on_generate)
buttons.append(gen_button)

# Prepare background surfaces
canvas_surface = pygame.Surface((SCREEN_W, SCREEN_H))
canvas_surface_dirty = True  # initially need to draw

# clock
clock = pygame.time.Clock()

# Right-panel positions (consistent layout)
label_x = CANVAS_LEFT + CANVAS_SIZE + 24   # left of labels
input_x = label_x + 140                   # input fields placed to the right of labels
input_w = 80
input_h = 30
base_y = 60
row_h = 44

# create NumericInput instances using same keys as params
inputs = {
    "Set_Speed": NumericInput(input_x, base_y + 0*row_h + 18, input_w, input_h, initial_value=500, step=10),
    "Set_low_AngVel": NumericInput(input_x, base_y + 1*row_h + 18, input_w, input_h, initial_value=200, step=10),
    "Set_Low_AngAcl": NumericInput(input_x, base_y + 2*row_h + 18, input_w, input_h, initial_value=1000, step=50),
    "Set_pri_offset": NumericInput(input_x, base_y + 3*row_h + 18, input_w, input_h, initial_value=22, step=1),
    "Set_post_offset": NumericInput(input_x, base_y + 4*row_h + 18, input_w, input_h, initial_value=0, step=1),
    "Set_K_SP": NumericInput(input_x, base_y + 5*row_h + 18, input_w, input_h, initial_value=0, step=0.001),
    "Set_C_SP": NumericInput(input_x, base_y + 6*row_h + 18, input_w, input_h, initial_value=0, step=0.5),
    "Set_Width": NumericInput(input_x, base_y + 7*row_h + 18, input_w, input_h, initial_value=86, step=1),
}
# スクロール範囲をラベルを含む行全体に広げる（行同士は重ならない）
for inp in inputs.values():
    inp.scroll_rect = pygame.Rect(label_x - 6, inp.rect.y - 14, inp.plus_rect.right + 6 - (label_x - 6), row_h)

# ファンの有無（入力欄の下の1行）
fan_y = base_y + len(params) * row_h + 4
fan_button = Button((label_x, fan_y, 140, 30), fan_label(), toggle_fan)
buttons.append(fan_button)
# スラロームの組の名前（空なら基本の組）。同じ速度・ファンで別の設計を "500_b" のように別に保存する
variant_label_x = label_x + 150
variant_box = TextBox((variant_label_x + 30, fan_y, 70, 30), allowed="abcdefghijklmnopqrstuvwxyz0123456789",
                      max_len=8)

# 保存 / 読込ボタン（右パネル、計算結果の下）
save_y = base_y + (len(params) + 1) * row_h + 162   # 計算結果の8行の下
save_button = Button((label_x, save_y, 140, 34), "保存 (Ctrl+S)", save_params)
load_button = Button((label_x + 150, save_y, 140, 34), "読込", load_params)
buttons.extend([save_button, load_button])

# 起動時: 初期プリセットを適用（保存データがあればそれも反映）
set_angle_preset(current_preset)

# main loop
running = True
while running:
    dt = clock.tick(60)
    for ev in pygame.event.get():
        if ev.type == pygame.QUIT:
            running = False
        if ev.type == pygame.KEYDOWN and ev.key == pygame.K_ESCAPE:
            running = False
        if ev.type == pygame.KEYDOWN:
            if ev.key == pygame.K_RETURN:     # Enter
                on_generate()
            elif ev.key == pygame.K_SPACE:
                on_generate(cp=True)
            elif ev.key == pygame.K_s and (ev.mod & (pygame.KMOD_CTRL | pygame.KMOD_META)):
                save_params()
        # forward events to widgets
        for b in buttons:
            b.handle_event(ev)
        for inp in inputs.values():
            inp.handle_event(ev)
        variant_box.handle_event(ev)

    # draw UI frame
    screen.fill(BG)
    # left panel
    pygame.draw.rect(screen, PANEL, (12, 12, 176, SCREEN_H-24))
    # right panel
    pygame.draw.rect(screen, PANEL, (CANVAS_LEFT + CANVAS_SIZE + 12, 12, SCREEN_W - (CANVAS_LEFT + CANVAS_SIZE + 24), SCREEN_H-24))

    # draw preset buttons
    for b in buttons:
        b.draw(screen)
    # 選択中のプリセットを枠で強調
    pygame.draw.rect(screen, (60, 60, 200), preset_buttons[current_preset].rect, 2, border_radius=6)

    # draw numeric inputs (right panel)
    for name, inp in inputs.items():
        inp.draw(screen)
    screen.blit(FONT.render("組", True, TEXT_COLOR), (variant_label_x, fan_y + 6))
    variant_box.draw(screen)

    # draw labels (right panel) aligned with inputs
    title = BIGFONT.render("たーんしみゅれーた (pygame)", True, TEXT_COLOR)
    screen.blit(title, (CANVAS_LEFT, 6))

    for i, (label, default, key) in enumerate(params):
        y = base_y + i*row_h
        lab = FONT.render(label, True, TEXT_COLOR)
        screen.blit(lab, (label_x, y + 6))
        # no old textboxes drawn anymore

    # draw canvas背景
    pygame.draw.rect(screen, (200,200,200), (CANVAS_LEFT-2, CANVAS_TOP-2, CANVAS_SIZE+4, CANVAS_SIZE+4), border_radius=4)
    draw_canvas(screen)

    # 軌道を毎フレーム描画
    draw_trace(screen)

    # draw info_lines on right panel (below inputs)
    info_x = label_x
    info_y = base_y + (len(params) + 1) * row_h + 4  # place below the fan toggle
    line_h = 18
    for i, line in enumerate(info_lines):
        txt = FONT.render(line, True, TEXT_COLOR)
        screen.blit(txt, (info_x, info_y + i*line_h))

    # 保存状態の表示
    ty = save_y + 44
    if status_msg:
        screen.blit(FONT.render(status_msg, True, TEXT_COLOR), (label_x, ty))
    speeds = saved_speeds_for_current()
    saved_txt = f"{preset_name(current_preset)} 保存済み: " + (", ".join(speeds) if speeds else "なし")
    screen.blit(FONT.render(saved_txt, True, TEXT_COLOR), (label_x, ty + line_h + 4))
    screen.blit(FONT.render(os.path.basename(SAVE_PATH), True, GREY_LINE), (label_x, ty + 2*line_h + 8))

    # draw footer instructions
    inst = FONT.render("数値はクリックして入力、またはスクロールで増減（Option:×0.1 / Cmd:×10）。ESCで終了。", True, TEXT_COLOR)
    screen.blit(inst, (CANVAS_LEFT, CANVAS_TOP + CANVAS_SIZE + 48))

    pygame.display.flip()

pygame.quit()
sys.exit()
