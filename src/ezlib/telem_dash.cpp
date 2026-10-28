#include "ezlib/telem_dash.hpp"

#include "ezlib/auton_select.hpp"
#include "ezlib/boot_log.hpp"
#include "customs/dash_config.hpp"
#include "customs/objDeclare.hpp"
#include "ezlib/ui_pages.hpp"
#include "main.h"
#include "liblvgl/lvgl.h"

#include <algorithm>
#include <cmath>
#include <cstdio>
#include <cstring>
#include <cstdint>
#include <malloc.h>
#include <unistd.h>

extern "C" char HeapLimit[];

namespace telem_dash {
namespace {

inline lv_color_t col_bg() { return lv_color_hex(0x050506); }
inline lv_color_t col_card() { return lv_color_hex(0x0E1012); }
inline lv_color_t col_card2() { return lv_color_hex(0x16181C); }
inline lv_color_t col_accent() { return lv_color_hex(0x8B0008); }
inline lv_color_t col_accent_hi() { return lv_color_hex(0xC40010); }
inline lv_color_t col_text() { return lv_color_hex(0xF5F5F5); }
inline lv_color_t col_muted() { return lv_color_hex(0x7A8088); }
inline lv_color_t col_warn() { return lv_color_hex(0xF0A020); }
inline lv_color_t col_hot() { return lv_color_hex(0xFF1A1A); }
inline lv_color_t col_dim() { return lv_color_hex(0x22262C); }
inline lv_color_t col_arc_bg() { return lv_color_hex(0x1A080A); }
inline lv_color_t col_edge() { return lv_color_hex(0x3A0004); }
// Neon / night-glow palette
inline lv_color_t col_neon_g() { return lv_color_hex(0x22FF66); }
inline lv_color_t col_neon_r() { return lv_color_hex(0xFF2040); }
inline lv_color_t col_neon_y() { return lv_color_hex(0xFFD400); }
inline lv_color_t col_gray() { return lv_color_hex(0x6A6E76); }
// 2017 Civic Type R 儀表黃針
inline lv_color_t col_type_r_needle() { return lv_color_hex(0xFFD400); }

lv_obj_t* parent_ = nullptr;

lv_obj_t* tach_ = nullptr; // boot 中大錶；live = 電量（無針）
lv_obj_t* face_ = nullptr;
lv_obj_t* needle_ = nullptr; // 只喺 boot 中錶掃黃針
lv_obj_t* needle_hub_ = nullptr;
lv_point_t needle_pts_[2] = {};
lv_coord_t needle_cx_ = 0;
lv_coord_t needle_cy_ = 0;
lv_coord_t needle_len_ = 60;
lv_coord_t needle_thick_ = 9;
lv_obj_t* tick_lo_ = nullptr; // 啟動中錶：刻度 0 / 6 / 12
lv_obj_t* tick_mid_ = nullptr;
lv_obj_t* tick_hi_ = nullptr;
lv_obj_t* tach_val_ = nullptr;
lv_obj_t* tach_unit_ = nullptr;
lv_obj_t* boot_tag_ = nullptr; // 主 Dash = BATT；啟動時隱藏
lv_obj_t* brand_ytt_ = nullptr;
lv_obj_t* brand_type_ = nullptr;

// 啟動左：細 torque（黃圈 + Nm）
lv_obj_t* torque_ = nullptr;
lv_obj_t* torque_hub_ = nullptr;
// 啟動右：細油量
lv_obj_t* fuel_ = nullptr;
lv_obj_t* fuel_val_ = nullptr;
lv_obj_t* fuel_unit_ = nullptr;
lv_obj_t* fuel_tag_ = nullptr;
char cache_fuel_[8] = {};
lv_coord_t torque_cx_ = 0;
lv_coord_t torque_cy_ = 0;

// 啟動 cluster（對齊 honda_civic_type_r_digital_dashboard_480x242.html）
lv_obj_t* splash_ = nullptr;
lv_obj_t* splash_title_ = nullptr;
lv_obj_t* splash_sub_ = nullptr;
lv_obj_t* boot_gear_ = nullptr;
lv_obj_t* boot_mode_ = nullptr;
lv_obj_t* boot_boost_ = nullptr;
lv_obj_t* boot_status_ = nullptr;
lv_obj_t* boot_turbo_ = nullptr;
lv_obj_t* boot_rpm_lbl_ = nullptr;
lv_obj_t* boot_nm_tag_ = nullptr; // TORQUE
lv_obj_t* boot_nm_val_ = nullptr; // 數字
lv_obj_t* boot_nm_unit_ = nullptr; // Nm
lv_obj_t* boot_leds_[7] = {};
char cache_boost_[24] = {};
char cache_nm_[16] = {};
constexpr int k_torque_peak_nm = 400; // Type R 風格峰值（動畫假數）

// Arc：rotation 135°、掃過 270°（0°=三點鐘，順時針）
constexpr int k_arc_rot_deg = 135;
constexpr int k_arc_sweep_deg = 270;
constexpr lv_coord_t k_needle_thick_boot = 9; // torque bar
constexpr lv_coord_t k_needle_thick_live = 5;
constexpr lv_coord_t k_arc_width_boot = 18;
constexpr lv_coord_t k_arc_width_live = 14;

lv_obj_t* hdr_auton_ = nullptr;
lv_obj_t* hdr_ram_ = nullptr;

lv_obj_t* pose_x_ = nullptr;
lv_obj_t* pose_y_ = nullptr;
lv_obj_t* pose_t_ = nullptr;
lv_obj_t* path_vals_ = nullptr;

lv_obj_t* dist_f_ = nullptr;
lv_obj_t* dist_b_ = nullptr;
lv_obj_t* dist_l_ = nullptr;
lv_obj_t* dist_r_ = nullptr;

lv_obj_t* drive_vals_ = nullptr;
lv_obj_t* mech_vals_ = nullptr;

char cache_auton_[40] = {};
char cache_ram_[24] = {};
char cache_px_[24] = {};
char cache_py_[24] = {};
char cache_pt_[24] = {};
char cache_path_[32] = {};
char cache_df_[28] = {};
char cache_db_[28] = {};
char cache_dl_[28] = {};
char cache_dr_[28] = {};
char cache_drive_[48] = {};
char cache_mech_[40] = {};
char cache_tach_[16] = {};

bool live_ready_ = false;
bool session_booted_ = false; // skip center boot after first calibrate this power-on
bool gauge_hidden_ = false;   // PID overlay 蓋住時 true
lv_coord_t gauge_sz_ = 168;
char last_error_[48] = {};

void set_err(const char* msg) {
    if (msg == nullptr) {
        last_error_[0] = '\0';
        return;
    }
    std::snprintf(last_error_, sizeof(last_error_), "%s", msg);
}

void style_text(lv_obj_t* lab, lv_color_t color) {
    if (lab == nullptr) return;
    lv_obj_set_style_text_color(lab, color, 0);
}

void style_font(lv_obj_t* lab, const lv_font_t* font) {
    if (lab == nullptr || font == nullptr) return;
    lv_obj_set_style_text_font(lab, font, 0);
}

lv_obj_t* make_label(lv_obj_t* parent, const char* text, lv_coord_t x, lv_coord_t y, lv_color_t color,
                     const lv_font_t* font = &lv_font_montserrat_12) {
    lv_obj_t* lab = lv_label_create(parent);
    if (lab == nullptr) return nullptr;
    lv_label_set_text(lab, text);
    lv_obj_set_pos(lab, x, y);
    style_text(lab, color);
    style_font(lab, font);
    return lab;
}

lv_obj_t* make_card(lv_obj_t* parent, lv_coord_t x, lv_coord_t y, lv_coord_t w, lv_coord_t h, lv_color_t bg) {
    lv_obj_t* card = lv_obj_create(parent);
    if (card == nullptr) return nullptr;
    lv_obj_set_pos(card, x, y);
    lv_obj_set_size(card, w, h);
    lv_obj_set_style_bg_color(card, bg, 0);
    lv_obj_set_style_bg_opa(card, LV_OPA_COVER, 0);
    lv_obj_set_style_radius(card, 10, 0); // soft Type R cards
    lv_obj_set_style_border_width(card, 1, 0);
    lv_obj_set_style_border_color(card, col_edge(), 0);
    lv_obj_set_style_pad_all(card, 0, 0);
    lv_obj_clear_flag(card, LV_OBJ_FLAG_SCROLLABLE);
    return card;
}

bool set_cached(lv_obj_t* lab, char* cache, std::size_t n, const char* text) {
    if (lab == nullptr || text == nullptr) return false;
    if (std::strncmp(cache, text, n) == 0) return false;
    std::snprintf(cache, n, "%s", text);
    lv_label_set_text(lab, cache);
    return true;
}

lv_color_t temp_color(double c) {
    if (c >= 50.0) return col_hot();
    if (c >= 42.0) return col_warn();
    return col_text();
}

// DIST: inch≈999 → gray; conf>50 green; conf<20 yellow; else red.
lv_color_t dist_color(float inches, int conf) {
    if (inches >= 998.5f || inches < 0.0f) return col_gray();
    if (conf > 50) return col_neon_g();
    if (conf < 20) return col_neon_y();
    return col_neon_r();
}

// Batt: <40 yellow, 40–80 red, >80 green.
lv_color_t batt_color(int pct) {
    if (pct > 80) return col_neon_g();
    if (pct < 40) return col_neon_y();
    return col_neon_r();
}

void set_hidden(lv_obj_t* o, bool hide) {
    if (o == nullptr) return;
    if (hide) lv_obj_add_flag(o, LV_OBJ_FLAG_HIDDEN);
    else lv_obj_clear_flag(o, LV_OBJ_FLAG_HIDDEN);
}

void place_tick(lv_obj_t* lab, lv_coord_t cx, lv_coord_t cy, lv_coord_t r, int pct) {
    if (lab == nullptr) return;
    const float deg =
        static_cast<float>(k_arc_rot_deg) +
        static_cast<float>(pct) * static_cast<float>(k_arc_sweep_deg) / 100.f;
    const float rad = deg * 3.14159265f / 180.f;
    const lv_coord_t x = cx + static_cast<lv_coord_t>(std::cos(rad) * static_cast<float>(r)) - 8;
    const lv_coord_t y = cy + static_cast<lv_coord_t>(std::sin(rad) * static_cast<float>(r)) - 6;
    lv_obj_set_pos(lab, x, y);
    lv_obj_move_foreground(lab);
}

void style_needle_bar(lv_color_t c, lv_coord_t thick) {
    needle_thick_ = thick;
    if (needle_ == nullptr) return;
    lv_obj_set_style_line_color(needle_, c, 0);
    lv_obj_set_style_line_width(needle_, thick, 0);
    lv_obj_set_style_line_rounded(needle_, true, 0);
}

void layout_needle(lv_coord_t arc_sz, lv_coord_t arc_w, bool long_needle) {
    if (tach_ == nullptr || needle_ == nullptr) return;
    const lv_coord_t ax = lv_obj_get_x(tach_);
    const lv_coord_t ay = lv_obj_get_y(tach_);
    needle_cx_ = ax + arc_sz / 2;
    needle_cy_ = ay + arc_sz / 2;

    lv_coord_t len = long_needle ? (arc_sz / 2 - arc_w - 2) : (arc_sz / 2 - 28);
    if (len < 40) len = 40;
    needle_len_ = len;
    style_needle_bar(long_needle ? col_type_r_needle() : col_text(),
                     long_needle ? k_needle_thick_boot : k_needle_thick_live);

    // line 覆蓋全屏，用絕對座標畫 bar（唔靠 transform，唔會被 clip 成一粒點）
    lv_obj_set_pos(needle_, 0, 0);
    lv_obj_set_size(needle_, 480, 240);
    lv_obj_move_foreground(needle_);

    if (needle_hub_ != nullptr) {
        const lv_coord_t hub = long_needle ? 16 : 12;
        lv_obj_set_size(needle_hub_, hub, hub);
        lv_obj_set_pos(needle_hub_, needle_cx_ - hub / 2, needle_cy_ - hub / 2);
        lv_obj_move_foreground(needle_hub_);
    }
}

void set_needle_pct(int pct) {
    if (needle_ == nullptr) return;
    if (pct < 0) pct = 0;
    if (pct > 100) pct = 100;
    const float deg =
        static_cast<float>(k_arc_rot_deg) +
        static_cast<float>(pct) * static_cast<float>(k_arc_sweep_deg) / 100.f;
    const float rad = deg * 3.14159265f / 180.f;
    // 由 hub 外沿開始，避免中心大數字完全遮住 bar 根部
    constexpr float k_inner = 10.f;
    const float c = std::cos(rad);
    const float s = std::sin(rad);
    needle_pts_[0].x = needle_cx_ + static_cast<lv_coord_t>(c * k_inner);
    needle_pts_[0].y = needle_cy_ + static_cast<lv_coord_t>(s * k_inner);
    needle_pts_[1].x = needle_cx_ + static_cast<lv_coord_t>(c * static_cast<float>(needle_len_));
    needle_pts_[1].y = needle_cy_ + static_cast<lv_coord_t>(s * static_cast<float>(needle_len_));
    lv_line_set_points(needle_, needle_pts_, 2);
}

void set_tach_pct(int pct, const char* center_text) {
    if (pct < 0) pct = 0;
    if (pct > 100) pct = 100;
    const bool boot = !live_ready_;
    const lv_color_t arc_c = boot ? col_accent_hi() : batt_color(pct);
    if (tach_ != nullptr) {
        lv_arc_set_value(tach_, pct);
        lv_obj_set_style_arc_color(tach_, arc_c, LV_PART_INDICATOR);
    }
    if (boot) {
        set_needle_pct(pct);
        style_needle_bar(col_type_r_needle(), k_needle_thick_boot);
        if (needle_hub_ != nullptr) {
            lv_obj_set_style_bg_color(needle_hub_, col_type_r_needle(), 0);
            lv_obj_set_style_border_color(needle_hub_, col_text(), 0);
        }
        if (tach_val_ != nullptr) style_text(tach_val_, col_text());
    } else {
        // 主 Dash 電量圈：淨弧＋數字，無指針
        set_hidden(needle_, true);
        set_hidden(needle_hub_, true);
        if (tach_val_ != nullptr) style_text(tach_val_, arc_c);
        if (tach_unit_ != nullptr) style_text(tach_unit_, arc_c);
    }
    set_cached(tach_val_, cache_tach_, sizeof(cache_tach_), center_text);
}

// 主 Dash：數字在弧心（啟動動畫專用 below 唔用喺呢度）
void center_tach_readout() {
    if (tach_ == nullptr) return;
    if (tach_val_ != nullptr) {
        lv_obj_align_to(tach_val_, tach_, LV_ALIGN_CENTER, 0, -4);
        lv_obj_move_foreground(tach_val_);
    }
    if (tach_unit_ != nullptr) {
        lv_obj_align_to(tach_unit_, tach_, LV_ALIGN_CENTER, 0, 18);
        lv_obj_move_foreground(tach_unit_);
    }
    if (boot_tag_ != nullptr) {
        lv_obj_align_to(boot_tag_, tach_, LV_ALIGN_CENTER, 0, 36);
        lv_obj_move_foreground(boot_tag_);
    }
}

void apply_arc_width(lv_coord_t w) {
    if (tach_ == nullptr) return;
    lv_obj_set_style_arc_width(tach_, w, LV_PART_MAIN);
    lv_obj_set_style_arc_width(tach_, w, LV_PART_INDICATOR);
}

void set_boot_cluster_visible(bool on) {
    set_hidden(face_, !on);
    set_hidden(tick_lo_, !on);
    set_hidden(tick_mid_, !on);
    set_hidden(tick_hi_, !on);
    set_hidden(boot_gear_, !on);
    set_hidden(boot_mode_, !on);
    set_hidden(boot_boost_, !on);
    set_hidden(boot_status_, !on);
    set_hidden(boot_turbo_, !on);
    set_hidden(boot_rpm_lbl_, !on);
    set_hidden(boot_nm_tag_, !on);
    set_hidden(boot_nm_val_, !on);
    set_hidden(boot_nm_unit_, !on);
    set_hidden(torque_, !on);
    set_hidden(torque_hub_, true); // 永遠唔顯示黃點
    set_hidden(fuel_, !on);
    set_hidden(fuel_val_, !on);
    set_hidden(fuel_unit_, !on);
    set_hidden(fuel_tag_, !on);
    for (lv_obj_t* led : boot_leds_) set_hidden(led, !on);
    if (on) style_needle_bar(col_type_r_needle(), k_needle_thick_boot);
}

void set_splash_visible(bool on) {
    set_hidden(splash_, !on);
    set_hidden(splash_title_, !on);
    set_hidden(splash_sub_, !on);
    if (on && splash_ != nullptr) lv_obj_move_foreground(splash_);
    if (on && splash_title_ != nullptr) lv_obj_move_foreground(splash_title_);
    if (on && splash_sub_ != nullptr) lv_obj_move_foreground(splash_sub_);
}

void update_boot_leds(float factor) {
    // HTML thresholds ≈ 3500…7800 / 8000 → 0.44…0.975
    constexpr float th[7] = {0.44f, 0.56f, 0.69f, 0.79f, 0.88f, 0.94f, 0.98f};
    for (int i = 0; i < 7; ++i) {
        lv_obj_t* led = boot_leds_[i];
        if (led == nullptr) continue;
        const bool on = factor >= th[i];
        lv_color_t c = col_dim();
        if (on) {
            if (i < 3) c = col_neon_g();
            else if (i < 5) c = col_neon_y();
            else c = col_neon_r();
        }
        lv_obj_set_style_bg_color(led, c, 0);
    }
}

// 啟動：中大錶（原本 KM/H + 黃針）+ 左右細圈（torque / fuel）
void place_boot_triple() {
    constexpr lv_coord_t mid_sz = 148;
    constexpr lv_coord_t side_sz = 100;
    constexpr lv_coord_t mid_y = 32;
    constexpr lv_coord_t side_y = 56;
    constexpr lv_coord_t mid_x = (480 - mid_sz) / 2; // 166
    constexpr lv_coord_t left_x = 8;
    constexpr lv_coord_t right_x = 480 - 8 - side_sz; // 372
    constexpr lv_coord_t side_arc_w = 12;
    gauge_sz_ = mid_sz;
    set_splash_visible(false);
    set_boot_cluster_visible(true);
    apply_arc_width(k_arc_width_boot);
    set_hidden(brand_ytt_, true);
    set_hidden(brand_type_, true);
    set_hidden(boot_tag_, true);
    set_hidden(tach_val_, false);
    set_hidden(tach_unit_, false);
    // torque／中錶都唔用針（針之前錯位）
    set_hidden(needle_, true);
    set_hidden(needle_hub_, true);

    // —— 中：原本大錶（弧 + KM/H，無針）——
    if (tach_ != nullptr) {
        lv_obj_set_size(tach_, mid_sz, mid_sz);
        lv_obj_set_pos(tach_, mid_x, mid_y);
    }
    const lv_coord_t cx = mid_x + mid_sz / 2;
    const lv_coord_t cy = mid_y + mid_sz / 2;
    if (face_ != nullptr) {
        const lv_coord_t face_sz = mid_sz - 2 * k_arc_width_boot - 8;
        lv_obj_set_size(face_, face_sz, face_sz);
        lv_obj_set_pos(face_, cx - face_sz / 2, cy - face_sz / 2);
    }
    if (tick_lo_ != nullptr) lv_label_set_text(tick_lo_, "0");
    if (tick_mid_ != nullptr) lv_label_set_text(tick_mid_, "6");
    if (tick_hi_ != nullptr) lv_label_set_text(tick_hi_, "12");
    place_tick(tick_lo_, cx, cy, mid_sz / 2 - k_arc_width_boot / 2 - 2, 0);
    place_tick(tick_mid_, cx, cy, mid_sz / 2 - k_arc_width_boot / 2 - 2, 50);
    place_tick(tick_hi_, cx, cy, mid_sz / 2 - k_arc_width_boot / 2 - 2, 100);
    if (tach_val_ != nullptr) {
        style_font(tach_val_, &lv_font_montserrat_48);
        style_text(tach_val_, col_text());
        lv_obj_align_to(tach_val_, tach_, LV_ALIGN_CENTER, 0, 8);
        lv_obj_move_foreground(tach_val_);
    }
    if (tach_unit_ != nullptr) {
        lv_label_set_text(tach_unit_, "KM/H");
        style_text(tach_unit_, col_accent_hi());
        lv_obj_align_to(tach_unit_, tach_val_, LV_ALIGN_OUT_BOTTOM_MID, 0, 0);
        lv_obj_move_foreground(tach_unit_);
    }

    // —— 左：細 torque（黃圈下 Nm）——
    torque_cx_ = left_x + side_sz / 2;
    torque_cy_ = side_y + side_sz / 2;
    if (torque_ != nullptr) {
        lv_obj_set_size(torque_, side_sz, side_sz);
        lv_obj_set_pos(torque_, left_x, side_y);
        lv_obj_set_style_arc_width(torque_, side_arc_w, LV_PART_MAIN);
        lv_obj_set_style_arc_width(torque_, side_arc_w, LV_PART_INDICATOR);
        lv_obj_set_style_arc_color(torque_, col_type_r_needle(), LV_PART_INDICATOR);
        set_hidden(torque_, false);
    }
    set_hidden(torque_hub_, true);
    if (boot_nm_tag_ != nullptr) {
        lv_label_set_text(boot_nm_tag_, "TORQUE");
        style_text(boot_nm_tag_, col_muted());
        lv_obj_set_pos(boot_nm_tag_, left_x + 18, side_y - 14);
        lv_obj_move_foreground(boot_nm_tag_);
    }
    // Nm 喺 torque 弧心偏下（無黃點）
    if (boot_nm_val_ != nullptr) {
        style_text(boot_nm_val_, col_type_r_needle());
        style_font(boot_nm_val_, &lv_font_montserrat_20);
        lv_obj_update_layout(boot_nm_val_);
        lv_obj_set_pos(boot_nm_val_, torque_cx_ - lv_obj_get_width(boot_nm_val_) / 2, torque_cy_ - 6);
        lv_obj_move_foreground(boot_nm_val_);
    }
    if (boot_nm_unit_ != nullptr) {
        style_text(boot_nm_unit_, col_type_r_needle());
        style_font(boot_nm_unit_, &lv_font_montserrat_12);
        lv_obj_align_to(boot_nm_unit_, boot_nm_val_, LV_ALIGN_OUT_BOTTOM_MID, 0, -2);
        lv_obj_move_foreground(boot_nm_unit_);
    }

    // —— 右：細油量 ——
    if (fuel_ != nullptr) {
        lv_obj_set_size(fuel_, side_sz, side_sz);
        lv_obj_set_pos(fuel_, right_x, side_y);
        lv_obj_set_style_arc_width(fuel_, side_arc_w, LV_PART_MAIN);
        lv_obj_set_style_arc_width(fuel_, side_arc_w, LV_PART_INDICATOR);
        set_hidden(fuel_, false);
    }
    if (fuel_val_ != nullptr) {
        style_font(fuel_val_, &lv_font_montserrat_24);
        style_text(fuel_val_, col_text());
        lv_obj_align_to(fuel_val_, fuel_, LV_ALIGN_CENTER, 0, -2);
        lv_obj_move_foreground(fuel_val_);
        set_hidden(fuel_val_, false);
    }
    if (fuel_unit_ != nullptr) {
        lv_label_set_text(fuel_unit_, "%");
        style_text(fuel_unit_, col_muted());
        lv_obj_align_to(fuel_unit_, fuel_val_, LV_ALIGN_OUT_BOTTOM_MID, 0, 0);
        lv_obj_move_foreground(fuel_unit_);
        set_hidden(fuel_unit_, false);
    }
    if (fuel_tag_ != nullptr) {
        lv_label_set_text(fuel_tag_, "FUEL");
        style_text(fuel_tag_, col_muted());
        lv_obj_set_pos(fuel_tag_, right_x + 30, side_y - 14);
        lv_obj_move_foreground(fuel_tag_);
        set_hidden(fuel_tag_, false);
    }
}

void place_tach_left_batt() {
    // 主 Dash：電量弧（無針）；關 boot 左右細錶
    constexpr lv_coord_t sz = 152;
    constexpr lv_coord_t x = 6;
    constexpr lv_coord_t y = 26;
    gauge_sz_ = sz;
    set_splash_visible(false);
    set_boot_cluster_visible(false);
    apply_arc_width(k_arc_width_live);
    set_hidden(boot_tag_, false);
    set_hidden(tach_val_, false);
    set_hidden(tach_unit_, false);
    set_hidden(needle_, true);
    set_hidden(needle_hub_, true);
    set_hidden(torque_, true);
    set_hidden(torque_hub_, true);
    set_hidden(fuel_, true);
    set_hidden(fuel_val_, true);
    set_hidden(fuel_unit_, true);
    set_hidden(fuel_tag_, true);

    if (tach_ != nullptr) {
        lv_obj_set_size(tach_, sz, sz);
        lv_obj_set_pos(tach_, x, y);
    }
    if (tach_unit_ != nullptr) {
        lv_label_set_text(tach_unit_, "%");
        style_text(tach_unit_, col_muted());
    }
    if (boot_tag_ != nullptr) {
        lv_label_set_text(boot_tag_, "BATT");
        style_text(boot_tag_, col_muted());
    }
    center_tach_readout();
    if (brand_ytt_ != nullptr) {
        set_hidden(brand_ytt_, false);
        lv_obj_set_pos(brand_ytt_, 8, 4);
    }
    if (brand_type_ != nullptr) {
        set_hidden(brand_type_, false);
        lv_obj_set_pos(brand_type_, 42, 6);
    }
}

void style_tach_arc(lv_obj_t* arc) {
    if (arc == nullptr) return;
    lv_arc_set_rotation(arc, 135);
    lv_arc_set_bg_angles(arc, 0, 270);
    lv_arc_set_range(arc, 0, 100);
    lv_arc_set_value(arc, 0);
    lv_obj_remove_style(arc, nullptr, LV_PART_KNOB);
    lv_obj_clear_flag(arc, LV_OBJ_FLAG_CLICKABLE);
    lv_obj_set_style_arc_width(arc, k_arc_width_live, LV_PART_MAIN);
    lv_obj_set_style_arc_width(arc, k_arc_width_live, LV_PART_INDICATOR);
    lv_obj_set_style_arc_color(arc, col_arc_bg(), LV_PART_MAIN);
    lv_obj_set_style_arc_color(arc, col_accent_hi(), LV_PART_INDICATOR);
    lv_obj_set_style_arc_rounded(arc, true, LV_PART_MAIN);
    lv_obj_set_style_arc_rounded(arc, true, LV_PART_INDICATOR);
}

lv_obj_t* make_led(lv_obj_t* parent, lv_coord_t x, lv_coord_t y) {
    lv_obj_t* led = lv_obj_create(parent);
    if (led == nullptr) return nullptr;
    lv_obj_set_pos(led, x, y);
    lv_obj_set_size(led, 10, 10);
    lv_obj_clear_flag(led, LV_OBJ_FLAG_SCROLLABLE | LV_OBJ_FLAG_CLICKABLE);
    lv_obj_set_style_radius(led, LV_RADIUS_CIRCLE, 0);
    lv_obj_set_style_bg_color(led, col_dim(), 0);
    lv_obj_set_style_bg_opa(led, LV_OPA_COVER, 0);
    lv_obj_set_style_border_width(led, 1, 0);
    lv_obj_set_style_border_color(led, col_edge(), 0);
    lv_obj_set_style_pad_all(led, 0, 0);
    lv_obj_add_flag(led, LV_OBJ_FLAG_HIDDEN);
    return led;
}

void create_boot_cluster(lv_obj_t* parent) {
    // Splash（自動播，唔使撳掣）
    splash_ = lv_obj_create(parent);
    if (splash_ != nullptr) {
        lv_obj_set_pos(splash_, 0, 0);
        lv_obj_set_size(splash_, 480, 240);
        lv_obj_clear_flag(splash_, LV_OBJ_FLAG_SCROLLABLE | LV_OBJ_FLAG_CLICKABLE);
        lv_obj_set_style_bg_color(splash_, lv_color_hex(0x000000), 0);
        lv_obj_set_style_bg_opa(splash_, LV_OPA_COVER, 0);
        lv_obj_set_style_border_width(splash_, 0, 0);
        lv_obj_set_style_radius(splash_, 0, 0);
        lv_obj_set_style_pad_all(splash_, 0, 0);
        lv_obj_add_flag(splash_, LV_OBJ_FLAG_HIDDEN);
    }
    splash_title_ = make_label(parent, "EzLib", 175, 95, col_text(), &lv_font_montserrat_16);
    splash_sub_ = make_label(parent, "SYSTEM CHECK", 185, 120, col_accent_hi(), &lv_font_montserrat_12);
    set_hidden(splash_title_, true);
    set_hidden(splash_sub_, true);

    // 頂列：N / SPORT / LEDs / boost
    boot_gear_ = make_label(parent, "N", 8, 4, col_text(), &lv_font_montserrat_14);
    boot_mode_ = make_label(parent, "SPORT", 28, 6, col_muted(), &lv_font_montserrat_12);
    constexpr lv_coord_t led0 = 175;
    for (int i = 0; i < 7; ++i) boot_leds_[i] = make_led(parent, led0 + i * 14, 6);
    boot_boost_ = make_label(parent, "0.0 BAR", 390, 6, col_accent_hi(), &lv_font_montserrat_12);

    // Torque 讀數（place_boot_dual 會移去左錶黃 hub 下）
    boot_nm_tag_ = make_label(parent, "TORQUE", 0, 0, col_muted(), &lv_font_montserrat_12);
    boot_nm_val_ = make_label(parent, "0", 0, 0, col_type_r_needle(), &lv_font_montserrat_20);
    boot_nm_unit_ = make_label(parent, "Nm", 0, 0, col_type_r_needle(), &lv_font_montserrat_14);

    // 底列
    boot_turbo_ = make_label(parent, "YKK TURBO", 8, 222, col_muted(), &lv_font_montserrat_12);
    boot_status_ = make_label(parent, "YTT IGNITION", 168, 222, col_accent_hi(), &lv_font_montserrat_12);
    boot_rpm_lbl_ = make_label(parent, "12000 RPM", 390, 222, col_muted(), &lv_font_montserrat_12);

    set_hidden(boot_gear_, true);
    set_hidden(boot_mode_, true);
    set_hidden(boot_boost_, true);
    set_hidden(boot_turbo_, true);
    set_hidden(boot_status_, true);
    set_hidden(boot_rpm_lbl_, true);
    set_hidden(boot_nm_tag_, true);
    set_hidden(boot_nm_val_, true);
    set_hidden(boot_nm_unit_, true);
}

void create_tach(lv_obj_t* parent) {
    create_boot_cluster(parent);

    tach_ = lv_arc_create(parent);
    style_tach_arc(tach_);

    torque_ = lv_arc_create(parent);
    style_tach_arc(torque_);
    if (torque_ != nullptr) {
        lv_obj_set_style_arc_color(torque_, col_type_r_needle(), LV_PART_INDICATOR);
        set_hidden(torque_, true);
    }
    torque_hub_ = lv_obj_create(parent);
    if (torque_hub_ != nullptr) {
        lv_obj_clear_flag(torque_hub_, LV_OBJ_FLAG_SCROLLABLE | LV_OBJ_FLAG_CLICKABLE);
        lv_obj_set_style_bg_color(torque_hub_, col_type_r_needle(), 0);
        lv_obj_set_style_bg_opa(torque_hub_, LV_OPA_COVER, 0);
        lv_obj_set_style_border_width(torque_hub_, 2, 0);
        lv_obj_set_style_border_color(torque_hub_, col_text(), 0);
        lv_obj_set_style_radius(torque_hub_, LV_RADIUS_CIRCLE, 0);
        lv_obj_set_style_pad_all(torque_hub_, 0, 0);
        set_hidden(torque_hub_, true);
    }

    fuel_ = lv_arc_create(parent);
    style_tach_arc(fuel_);
    if (fuel_ != nullptr) {
        lv_obj_set_style_arc_color(fuel_, col_neon_g(), LV_PART_INDICATOR);
        set_hidden(fuel_, true);
    }
    fuel_val_ = make_label(parent, "0", 0, 0, col_text(), &lv_font_montserrat_24);
    fuel_unit_ = make_label(parent, "%", 0, 0, col_muted(), &lv_font_montserrat_12);
    fuel_tag_ = make_label(parent, "FUEL", 0, 0, col_muted(), &lv_font_montserrat_12);
    set_hidden(fuel_val_, true);
    set_hidden(fuel_unit_, true);
    set_hidden(fuel_tag_, true);

    face_ = lv_obj_create(parent);
    if (face_ != nullptr) {
        lv_obj_clear_flag(face_, LV_OBJ_FLAG_SCROLLABLE | LV_OBJ_FLAG_CLICKABLE);
        lv_obj_set_style_bg_color(face_, lv_color_hex(0x08090D), 0);
        lv_obj_set_style_bg_opa(face_, LV_OPA_COVER, 0);
        lv_obj_set_style_border_width(face_, 1, 0);
        lv_obj_set_style_border_color(face_, col_edge(), 0);
        lv_obj_set_style_radius(face_, LV_RADIUS_CIRCLE, 0);
        lv_obj_set_style_pad_all(face_, 0, 0);
        lv_obj_add_flag(face_, LV_OBJ_FLAG_HIDDEN);
    }

    tick_lo_ = make_label(parent, "0", 0, 0, col_muted(), &lv_font_montserrat_12);
    tick_mid_ = make_label(parent, "6", 0, 0, col_muted(), &lv_font_montserrat_12);
    tick_hi_ = make_label(parent, "12", 0, 0, col_accent_hi(), &lv_font_montserrat_12);
    set_hidden(tick_lo_, true);
    set_hidden(tick_mid_, true);
    set_hidden(tick_hi_, true);

    needle_ = lv_line_create(parent);
    if (needle_ != nullptr) {
        lv_obj_clear_flag(needle_, LV_OBJ_FLAG_SCROLLABLE | LV_OBJ_FLAG_CLICKABLE);
        lv_obj_set_style_bg_opa(needle_, LV_OPA_TRANSP, 0);
        lv_obj_set_style_border_width(needle_, 0, 0);
        lv_obj_set_style_pad_all(needle_, 0, 0);
        style_needle_bar(col_type_r_needle(), k_needle_thick_boot);
    }
    needle_hub_ = lv_obj_create(parent);
    if (needle_hub_ != nullptr) {
        lv_obj_clear_flag(needle_hub_, LV_OBJ_FLAG_SCROLLABLE | LV_OBJ_FLAG_CLICKABLE);
        lv_obj_set_style_bg_color(needle_hub_, col_type_r_needle(), 0);
        lv_obj_set_style_bg_opa(needle_hub_, LV_OPA_COVER, 0);
        lv_obj_set_style_border_width(needle_hub_, 2, 0);
        lv_obj_set_style_border_color(needle_hub_, col_text(), 0);
        lv_obj_set_style_radius(needle_hub_, LV_RADIUS_CIRCLE, 0);
        lv_obj_set_style_pad_all(needle_hub_, 0, 0);
    }

    tach_val_ = make_label(parent, "0", 0, 0, col_text(), &lv_font_montserrat_48);
    tach_unit_ = make_label(parent, "%", 0, 0, col_muted(), &lv_font_montserrat_12);
    boot_tag_ = make_label(parent, "BATT", 0, 0, col_muted(), &lv_font_montserrat_12);
    // layout_needle 之後先有 cx/cy；create 時先唔畫
}

void poke_ui() {
    if (parent_ != nullptr) lv_obj_invalidate(parent_);
    lv_timer_handler();
}

// 自動啟動：splash → 高幀率 sin 掃針（唔使掣）
void run_boot_sweep_blocking() {
    boot_log::log("telem_dash:Type R auto ignition");

    // 1) Splash（縮短，快啲入動畫）
    set_boot_cluster_visible(false);
    set_hidden(tach_, true);
    set_hidden(needle_, true);
    set_hidden(needle_hub_, true);
    set_hidden(tach_val_, true);
    set_hidden(tach_unit_, true);
    set_hidden(brand_ytt_, true);
    set_hidden(brand_type_, true);
    set_splash_visible(true);
    poke_ui();
    pros::delay(700);

    // 2) 中大錶 + 左右細圈 sin sweep（全程無針）
    set_splash_visible(false);
    set_hidden(tach_, false);
    set_hidden(needle_, true);
    set_hidden(needle_hub_, true);
    set_hidden(tach_val_, false);
    set_hidden(tach_unit_, false);
    place_boot_triple();
    if (boot_status_ != nullptr) {
        lv_label_set_text(boot_status_, "YTT IGNITION");
        style_text(boot_status_, col_accent_hi());
    }
    if (tach_val_ != nullptr) lv_obj_align_to(tach_val_, tach_, LV_ALIGN_CENTER, 0, 8);
    if (tach_unit_ != nullptr) lv_obj_align_to(tach_unit_, tach_val_, LV_ALIGN_OUT_BOTTOM_MID, 0, 0);
    poke_ui();

    constexpr int k_steps = 90;
    constexpr int k_step_ms = 14;
    bool was_vtec = false;
    int last_pct = -1;
    for (int i = 0; i <= k_steps; ++i) {
        const float prog = static_cast<float>(i) / static_cast<float>(k_steps);
        const float factor = std::sin(prog * 3.14159265f);
        const int arc_pct = static_cast<int>(factor * 100.0f + 0.5f);
        const int speed = arc_pct;

        // 中：紅弧 + KM/H（無針）
        if (tach_ != nullptr) {
            lv_arc_set_value(tach_, arc_pct);
            lv_obj_set_style_arc_color(tach_, col_accent_hi(), LV_PART_INDICATOR);
        }

        // 左：torque 黃弧 + 黃圈下 Nm（無針）
        if (torque_ != nullptr) {
            lv_arc_set_value(torque_, arc_pct);
            lv_obj_set_style_arc_color(torque_, col_type_r_needle(), LV_PART_INDICATOR);
        }
        // 右：油量
        if (fuel_ != nullptr) {
            lv_arc_set_value(fuel_, arc_pct);
            lv_obj_set_style_arc_color(fuel_, batt_color(arc_pct), LV_PART_INDICATOR);
        }

        if (arc_pct != last_pct) {
            last_pct = arc_pct;
            char b[8];
            std::snprintf(b, sizeof(b), "%d", speed);
            set_cached(tach_val_, cache_tach_, sizeof(cache_tach_), b);
            if (tach_val_ != nullptr) {
                lv_obj_update_layout(tach_val_);
                lv_obj_align_to(tach_val_, tach_, LV_ALIGN_CENTER, 0, 8);
            }
            if (tach_unit_ != nullptr) {
                lv_obj_align_to(tach_unit_, tach_val_, LV_ALIGN_OUT_BOTTOM_MID, 0, 0);
            }

            const int nm = static_cast<int>(factor * static_cast<float>(k_torque_peak_nm) + 0.5f);
            char nb[16];
            std::snprintf(nb, sizeof(nb), "%d", nm);
            if (set_cached(boot_nm_val_, cache_nm_, sizeof(cache_nm_), nb) && boot_nm_val_ != nullptr) {
                lv_obj_update_layout(boot_nm_val_);
                lv_obj_set_pos(boot_nm_val_, torque_cx_ - lv_obj_get_width(boot_nm_val_) / 2, torque_cy_ - 6);
                if (boot_nm_unit_ != nullptr) {
                    lv_obj_align_to(boot_nm_unit_, boot_nm_val_, LV_ALIGN_OUT_BOTTOM_MID, 0, -2);
                }
            }

            char fb[8];
            std::snprintf(fb, sizeof(fb), "%d", arc_pct);
            if (set_cached(fuel_val_, cache_fuel_, sizeof(cache_fuel_), fb) && fuel_val_ != nullptr) {
                lv_obj_update_layout(fuel_val_);
                lv_obj_align_to(fuel_val_, fuel_, LV_ALIGN_CENTER, 0, -2);
                if (fuel_unit_ != nullptr) {
                    lv_obj_align_to(fuel_unit_, fuel_val_, LV_ALIGN_OUT_BOTTOM_MID, 0, 0);
                }
                style_text(fuel_val_, batt_color(arc_pct));
            }
        }
        if ((i & 1) == 0) update_boot_leds(factor);
        if (boot_boost_ != nullptr && (i % 3) == 0) {
            char bb[24];
            std::snprintf(bb, sizeof(bb), "%.1f BAR", static_cast<double>(factor * 1.3f));
            if (std::strncmp(cache_boost_, bb, sizeof(cache_boost_)) != 0) {
                std::snprintf(cache_boost_, sizeof(cache_boost_), "%s", bb);
                lv_label_set_text(boot_boost_, cache_boost_);
            }
        }
        const bool vtec = factor >= 0.72f;
        if (vtec != was_vtec && boot_turbo_ != nullptr) {
            was_vtec = vtec;
            lv_label_set_text(boot_turbo_, vtec ? "TURBO VTEC" : "TURBO STD");
            style_text(boot_turbo_, vtec ? col_accent_hi() : col_muted());
        }

        lv_timer_handler();
        lv_timer_handler();
        pros::delay(k_step_ms);
    }

    // 3) idle
    if (tach_ != nullptr) lv_arc_set_value(tach_, 11);
    if (torque_ != nullptr) lv_arc_set_value(torque_, 11);
    if (fuel_ != nullptr) lv_arc_set_value(fuel_, 11);
    set_cached(tach_val_, cache_tach_, sizeof(cache_tach_), "0");
    update_boot_leds(0.11f);
    if (boot_boost_ != nullptr) {
        std::snprintf(cache_boost_, sizeof(cache_boost_), "0.0 BAR");
        lv_label_set_text(boot_boost_, cache_boost_);
    }
    if (boot_nm_val_ != nullptr) {
        std::snprintf(cache_nm_, sizeof(cache_nm_), "0");
        lv_label_set_text(boot_nm_val_, cache_nm_);
        lv_obj_update_layout(boot_nm_val_);
        lv_obj_set_pos(boot_nm_val_, torque_cx_ - lv_obj_get_width(boot_nm_val_) / 2, torque_cy_ - 6);
    }
    set_hidden(needle_, true);
    set_hidden(needle_hub_, true);
    set_hidden(torque_hub_, true);
    if (fuel_val_ != nullptr) {
        std::snprintf(cache_fuel_, sizeof(cache_fuel_), "0");
        lv_label_set_text(fuel_val_, cache_fuel_);
    }
    if (boot_status_ != nullptr) {
        lv_label_set_text(boot_status_, "SYSTEM ONLINE");
        style_text(boot_status_, col_neon_g());
    }
    poke_ui();
    pros::delay(200);
}

void on_pid_btn(lv_event_t* e) {
    (void)e;
    ui_pages::show_pid();
}

void build_data_panels(lv_obj_t* parent) {
    hdr_auton_ = make_label(parent, "auton --", 120, 6, col_muted(), &lv_font_montserrat_12);
    hdr_ram_ = make_label(parent, "RAM:--", 380, 6, col_muted(), &lv_font_montserrat_12);

    lv_obj_t* pose_card = make_card(parent, 168, 26, 148, 152, col_card());
    if (pose_card != nullptr) {
        make_label(pose_card, "POSE", 8, 4, col_accent_hi(), &lv_font_montserrat_12);
        make_label(pose_card, "X", 8, 28, col_muted(), &lv_font_montserrat_12);
        make_label(pose_card, "Y", 8, 56, col_muted(), &lv_font_montserrat_12);
        make_label(pose_card, "T", 8, 84, col_muted(), &lv_font_montserrat_12);
        pose_x_ = make_label(pose_card, "---.-", 28, 26, col_text(), &lv_font_montserrat_16);
        pose_y_ = make_label(pose_card, "---.-", 28, 54, col_text(), &lv_font_montserrat_16);
        pose_t_ = make_label(pose_card, "---.-", 28, 82, col_accent_hi(), &lv_font_montserrat_16);
        path_vals_ = make_label(pose_card, "d ---.-", 8, 118, col_muted(), &lv_font_montserrat_14);
    }

    lv_obj_t* dist_card = make_card(parent, 324, 26, 148, 152, col_card());
    if (dist_card != nullptr) {
        make_label(dist_card, "DIST", 6, 4, col_accent_hi(), &lv_font_montserrat_12);
        // One label / cell, three lines: tag / inches / conf
        auto cell = [&](const char* tag, lv_coord_t x, lv_coord_t y, lv_obj_t*& out) {
            lv_obj_t* box = make_card(dist_card, x, y, 64, 60, col_card2());
            if (box == nullptr) return;
            char init[24];
            std::snprintf(init, sizeof(init), "%s\n--.-\n--", tag);
            out = make_label(box, init, 6, 4, col_text(), &lv_font_montserrat_12);
        };
        cell("F", 8, 22, dist_f_);
        cell("B", 76, 22, dist_b_);
        cell("L", 8, 86, dist_l_);
        cell("R", 76, 86, dist_r_);
    }

    lv_obj_t* temp_card = make_card(parent, 6, 184, 386, 26, col_card());
    if (temp_card != nullptr) {
        make_label(temp_card, "DRV", 4, 5, col_muted(), &lv_font_montserrat_12);
        drive_vals_ = make_label(temp_card, "--/--/-- --/--/--", 34, 5, col_text(), &lv_font_montserrat_12);
        make_label(temp_card, "MCH", 218, 5, col_muted(), &lv_font_montserrat_12);
        mech_vals_ = make_label(temp_card, "--/-- --/--", 248, 5, col_text(), &lv_font_montserrat_12);
    }

    lv_obj_t* pid_btn = lv_btn_create(parent);
    if (pid_btn != nullptr) {
        lv_obj_set_pos(pid_btn, 400, 184);
        lv_obj_set_size(pid_btn, 68, 26);
        lv_obj_set_style_bg_color(pid_btn, col_card2(), 0);
        lv_obj_set_style_border_width(pid_btn, 1, 0);
        lv_obj_set_style_border_color(pid_btn, col_accent_hi(), 0);
        lv_obj_set_style_radius(pid_btn, 8, 0);
        lv_obj_t* plab = lv_label_create(pid_btn);
        if (plab != nullptr) {
            lv_label_set_text(plab, "PID");
            style_font(plab, &lv_font_montserrat_14);
            style_text(plab, col_accent_hi());
            lv_obj_center(plab);
        }
        lv_obj_add_event_cb(pid_btn, on_pid_btn, LV_EVENT_CLICKED, nullptr);
    }
}

void layout_live() {
    place_tach_left_batt();
    if (parent_ != nullptr) build_data_panels(parent_);
    live_ready_ = true;
    const int batt = static_cast<int>(pros::battery::get_capacity());
    char b[8];
    std::snprintf(b, sizeof(b), "%d", batt);
    set_tach_pct(batt, b);
}

} // namespace

void build_chrome(lv_obj_t* parent) {
    lv_obj_set_style_bg_color(parent, col_bg(), 0);
    lv_obj_set_style_bg_opa(parent, LV_OPA_COVER, 0);
    brand_ytt_ = make_label(parent, "YTT", 200, 4, col_accent_hi(), &lv_font_montserrat_16);
    brand_type_ = make_label(parent, "EzLib", 234, 6, col_text(), &lv_font_montserrat_12);
    create_tach(parent);
}

void poke_refresh() {
    if (parent_ == nullptr) return;
    lv_obj_invalidate(parent_);
    for (int i = 0; i < 5; ++i) {
        lv_timer_handler();
        pros::delay(5);
    }
}

void start() {
    if (parent_ != nullptr) return;

    boot_log::log("telem_dash:start begin");
    live_ready_ = false;
    set_err("starting");

    if (lv_disp_get_default() == nullptr) {
        set_err("no disp");
        boot_log::log("telem_dash:start FAIL no disp");
        return;
    }
    parent_ = lv_scr_act();
    if (parent_ == nullptr) {
        set_err("no scr");
        boot_log::log("telem_dash:start FAIL no scr");
        return;
    }

    lv_obj_clean(parent_);
    build_chrome(parent_);
    if (!session_booted_) {
        // 等 finish_boot 自動播 ignition；先黑屏等 splash
        set_hidden(tach_, true);
        set_hidden(needle_, true);
        set_hidden(needle_hub_, true);
        set_hidden(tach_val_, true);
        set_hidden(tach_unit_, true);
        set_hidden(torque_, true);
        set_hidden(torque_hub_, true);
        set_hidden(fuel_, true);
        set_hidden(fuel_val_, true);
        set_hidden(fuel_unit_, true);
        set_hidden(fuel_tag_, true);
        set_hidden(boot_tag_, true);
        set_hidden(brand_ytt_, true);
        set_hidden(brand_type_, true);
        set_boot_cluster_visible(false);
        boot_log::log("telem_dash:await auto ignition");
    } else {
        layout_live();
        boot_log::log("telem_dash:live direct");
    }
    poke_refresh();
    set_err("");
}

void finish_boot() {
    if (!ready()) start();
    if (!ready() || live_ready_) return;

    boot_log::log("telem_dash:finish_boot");
    // 自動：splash → sin 掃針（唔使掣）
    run_boot_sweep_blocking();

    place_tach_left_batt();
    build_data_panels(parent_);
    live_ready_ = true;
    session_booted_ = true;

    const int batt = static_cast<int>(pros::battery::get_capacity());
    char b[8];
    std::snprintf(b, sizeof(b), "%d", batt);
    set_tach_pct(batt, b);
    center_tach_readout();
    poke_refresh();
    boot_log::log("telem_dash:live ready");
}

void release() {
    if (parent_ == nullptr) return;
    boot_log::log("telem_dash:release");
    lv_obj_clean(parent_);
    parent_ = nullptr;
    tach_ = face_ = needle_ = needle_hub_ = nullptr;
    tick_lo_ = tick_mid_ = tick_hi_ = nullptr;
    tach_val_ = tach_unit_ = boot_tag_ = nullptr;
    torque_ = torque_hub_ = nullptr;
    fuel_ = fuel_val_ = fuel_unit_ = fuel_tag_ = nullptr;
    brand_ytt_ = brand_type_ = nullptr;
    splash_ = splash_title_ = splash_sub_ = nullptr;
    boot_gear_ = boot_mode_ = boot_boost_ = nullptr;
    boot_status_ = boot_turbo_ = boot_rpm_lbl_ = nullptr;
    boot_nm_tag_ = boot_nm_val_ = boot_nm_unit_ = nullptr;
    cache_nm_[0] = '\0';
    cache_fuel_[0] = '\0';
    for (lv_obj_t*& led : boot_leds_) led = nullptr;
    cache_boost_[0] = '\0';
    hdr_auton_ = hdr_ram_ = nullptr;
    pose_x_ = pose_y_ = pose_t_ = path_vals_ = nullptr;
    dist_f_ = dist_b_ = dist_l_ = dist_r_ = nullptr;
    drive_vals_ = mech_vals_ = nullptr;
    cache_auton_[0] = cache_ram_[0] = cache_tach_[0] = '\0';
    cache_px_[0] = cache_py_[0] = cache_pt_[0] = cache_path_[0] = '\0';
    cache_df_[0] = cache_db_[0] = cache_dl_[0] = cache_dr_[0] = '\0';
    cache_drive_[0] = cache_mech_[0] = '\0';
    live_ready_ = false;
    gauge_hidden_ = false;
}

void focus() {
    if (!ready()) start();
    else poke_refresh();
}

bool ready() { return parent_ != nullptr; }
bool live() { return live_ready_; }
const char* last_error() { return last_error_; }

void set_gauge_hidden(bool hidden) {
    gauge_hidden_ = hidden;
    // PID overlay：主 Dash 電量弧無針
    set_hidden(tach_, hidden);
    set_hidden(needle_, true);
    set_hidden(needle_hub_, true);
    set_hidden(tach_val_, hidden);
    set_hidden(tach_unit_, hidden);
    set_hidden(boot_tag_, hidden);
    if (hidden) {
        set_hidden(face_, true);
    } else if (live_ready_) {
        set_hidden(face_, true);
        set_hidden(tick_lo_, true);
        set_hidden(tick_mid_, true);
        set_hidden(tick_hi_, true);
        set_hidden(needle_, true);
        set_hidden(needle_hub_, true);
        center_tach_readout();
    }
}

void refresh() {
    if (parent_ == nullptr || !live_ready_ || gauge_hidden_) return;

    char buf[64];

    std::string auton = auton_select::name();
    if (auton.size() > 16) auton.resize(16);
    set_cached(hdr_auton_, cache_auton_, sizeof(cache_auton_), auton.c_str());

    // Left gauge = battery % (center readout + neon by level).
    const int batt = static_cast<int>(pros::battery::get_capacity());
    char bb[8];
    std::snprintf(bb, sizeof(bb), "%d", batt);
    set_tach_pct(batt, bb);
    center_tach_readout();

    const auto mi = mallinfo();
    std::size_t free_b = mi.fordblks;
    void* const brk = sbrk(0);
    if (brk != reinterpret_cast<void*>(-1)) {
        const auto brk_u = reinterpret_cast<std::uintptr_t>(brk);
        const auto lim_u = reinterpret_cast<std::uintptr_t>(HeapLimit);
        if (brk_u < lim_u) free_b += static_cast<std::size_t>(lim_u - brk_u);
    }
    const unsigned free_kb = static_cast<unsigned>(free_b / 1024U);
    if (free_kb >= 1024U) {
        std::snprintf(buf, sizeof(buf), "%.1fM", free_kb / 1024.0);
    } else {
        std::snprintf(buf, sizeof(buf), "%uk", free_kb);
    }
    set_cached(hdr_ram_, cache_ram_, sizeof(cache_ram_), buf);

    const auto pose = chassis.getPose();
    std::snprintf(buf, sizeof(buf), "%6.1f", pose.x);
    set_cached(pose_x_, cache_px_, sizeof(cache_px_), buf);
    std::snprintf(buf, sizeof(buf), "%6.1f", pose.y);
    set_cached(pose_y_, cache_py_, sizeof(cache_py_), buf);
    std::snprintf(buf, sizeof(buf), "%5.1f", pose.theta);
    set_cached(pose_t_, cache_pt_, sizeof(cache_pt_), buf);

    std::snprintf(buf, sizeof(buf), "d %5.1f", ezPID.get_traveled());
    set_cached(path_vals_, cache_path_, sizeof(cache_path_), buf);

    auto fill_dist = [](lv_obj_t* lab, char* cache, std::size_t n, const char* tag, pros::Distance& s,
                        float inches) {
        const int32_t c = s.get_confidence();
        const int conf = (c == PROS_ERR || c < 0 || c > 100) ? 0 : static_cast<int>(c);
        char b[32];
        std::snprintf(b, sizeof(b), "%s\n%.1f\n%d", tag, inches, conf);
        if (set_cached(lab, cache, n, b) || lab != nullptr) {
            style_text(lab, dist_color(inches, conf));
        }
    };
    fill_dist(dist_f_, cache_df_, sizeof(cache_df_), "F", dist_front, static_cast<float>(get_kf_dist(dist_front)));
    fill_dist(dist_b_, cache_db_, sizeof(cache_db_), "B", dist_back, static_cast<float>(get_kf_dist(dist_back)));
    fill_dist(dist_l_, cache_dl_, sizeof(cache_dl_), "L", dist_left, static_cast<float>(get_kf_dist(dist_left)));
    fill_dist(dist_r_, cache_dr_, sizeof(cache_dr_), "R", dist_right, static_cast<float>(get_kf_dist(dist_right)));

    // Temp sources: edit include/customs/dash_config.hpp
    double drive_max = 0;
    dash_config::format_drive_temps(buf, sizeof(buf), &drive_max);
    set_cached(drive_vals_, cache_drive_, sizeof(cache_drive_), buf);
    if (drive_vals_ != nullptr) style_text(drive_vals_, temp_color(drive_max));

    double mech_max = 0;
    dash_config::format_mech_temps(buf, sizeof(buf), &mech_max);
    set_cached(mech_vals_, cache_mech_, sizeof(cache_mech_), buf);
    if (mech_vals_ != nullptr) style_text(mech_vals_, temp_color(mech_max));
}

} // namespace telem_dash
