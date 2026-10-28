#include "ezlib/telem_dash.hpp"

#include "ezlib/auton_select.hpp"
#include "ezlib/boot_log.hpp"
#include "customs/dash_config.hpp"
#include "customs/objDeclare.hpp"
#include "ezlib/ui_pages.hpp"
#include "main.h"
#include "liblvgl/lvgl.h"

#include <algorithm>
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

lv_obj_t* parent_ = nullptr;

lv_obj_t* tach_ = nullptr;
lv_obj_t* tach_val_ = nullptr;
lv_obj_t* tach_unit_ = nullptr;
lv_obj_t* boot_tag_ = nullptr;
lv_obj_t* brand_ytt_ = nullptr;
lv_obj_t* brand_type_ = nullptr;

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

void set_tach_pct(int pct, const char* center_text) {
    if (pct < 0) pct = 0;
    if (pct > 100) pct = 100;
    const lv_color_t c = live_ready_ ? batt_color(pct) : col_accent_hi();
    if (tach_ != nullptr) {
        lv_arc_set_value(tach_, pct);
        lv_obj_set_style_arc_color(tach_, c, LV_PART_INDICATOR);
    }
    set_cached(tach_val_, cache_tach_, sizeof(cache_tach_), center_text);
    if (tach_val_ != nullptr) style_text(tach_val_, c);
    if (tach_unit_ != nullptr && live_ready_) style_text(tach_unit_, c);
}

void center_tach_readout() {
    // Keep % number dead-center in the arc ring.
    if (tach_ == nullptr) return;
    if (tach_val_ != nullptr) lv_obj_align_to(tach_val_, tach_, LV_ALIGN_CENTER, 0, -4);
    if (tach_unit_ != nullptr) lv_obj_align_to(tach_unit_, tach_, LV_ALIGN_CENTER, 0, 18);
    if (boot_tag_ != nullptr) lv_obj_align_to(boot_tag_, tach_, LV_ALIGN_CENTER, 0, 36);
}

void place_tach_center() {
    // Brain content ~480×240; center a large arc.
    constexpr lv_coord_t sz = 200;
    constexpr lv_coord_t x = (480 - sz) / 2; // 140
    constexpr lv_coord_t y = 18;
    if (tach_ != nullptr) {
        lv_obj_set_size(tach_, sz, sz);
        lv_obj_set_pos(tach_, x, y);
    }
    if (tach_unit_ != nullptr) lv_label_set_text(tach_unit_, "LOAD");
    if (boot_tag_ != nullptr) {
        lv_label_set_text(boot_tag_, "CAL");
        style_text(boot_tag_, col_accent_hi());
    }
    center_tach_readout();
    // No brand text during boot / calibrate.
    if (brand_ytt_ != nullptr) lv_obj_add_flag(brand_ytt_, LV_OBJ_FLAG_HIDDEN);
    if (brand_type_ != nullptr) lv_obj_add_flag(brand_type_, LV_OBJ_FLAG_HIDDEN);
}

void place_tach_left_batt() {
    constexpr lv_coord_t sz = 152;
    constexpr lv_coord_t x = 6;
    constexpr lv_coord_t y = 26;
    if (tach_ != nullptr) {
        lv_obj_set_size(tach_, sz, sz);
        lv_obj_set_pos(tach_, x, y);
    }
    if (tach_unit_ != nullptr) lv_label_set_text(tach_unit_, "%");
    if (boot_tag_ != nullptr) {
        lv_label_set_text(boot_tag_, "BATT");
        style_text(boot_tag_, col_muted());
    }
    center_tach_readout();
    if (brand_ytt_ != nullptr) {
        lv_obj_clear_flag(brand_ytt_, LV_OBJ_FLAG_HIDDEN);
        lv_obj_set_pos(brand_ytt_, 8, 4);
    }
    if (brand_type_ != nullptr) {
        lv_obj_clear_flag(brand_type_, LV_OBJ_FLAG_HIDDEN);
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
    lv_obj_set_style_arc_width(arc, 14, LV_PART_MAIN);
    lv_obj_set_style_arc_width(arc, 14, LV_PART_INDICATOR);
    lv_obj_set_style_arc_color(arc, col_arc_bg(), LV_PART_MAIN);
    lv_obj_set_style_arc_color(arc, col_accent_hi(), LV_PART_INDICATOR);
    lv_obj_set_style_arc_rounded(arc, false, LV_PART_MAIN);
    lv_obj_set_style_arc_rounded(arc, false, LV_PART_INDICATOR);
}

void create_tach(lv_obj_t* parent) {
    tach_ = lv_arc_create(parent);
    style_tach_arc(tach_);
    tach_val_ = make_label(parent, "0", 0, 0, col_text(), &lv_font_montserrat_48);
    tach_unit_ = make_label(parent, "LOAD", 0, 0, col_muted(), &lv_font_montserrat_12);
    boot_tag_ = make_label(parent, "CAL", 0, 0, col_accent_hi(), &lv_font_montserrat_12);
}

// Blocking 0→100% on the caller thread (~2s). Runs after calibrate in finish_boot
// so a fast IMU calibrate cannot skip the animation.
void run_boot_sweep_blocking() {
    boot_log::log("telem_dash:sweep 0→100 (~2s)");
    if (boot_tag_ != nullptr) {
        lv_label_set_text(boot_tag_, "IGN");
        style_text(boot_tag_, col_accent_hi());
    }
    for (int v = 0; v <= 100; v += 2) {
        char b[8];
        std::snprintf(b, sizeof(b), "%d", v);
        set_tach_pct(v, b);
        pros::delay(40); // 50 × 40ms = 2000ms
    }
    if (boot_tag_ != nullptr) {
        lv_label_set_text(boot_tag_, "CAL");
        style_text(boot_tag_, col_accent_hi());
    }
    pros::delay(150);
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
        place_tach_center();
        set_tach_pct(0, "0");
        boot_log::log("telem_dash:boot centered (await finish_boot sweep)");
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
    place_tach_center();
    poke_refresh();
    run_boot_sweep_blocking();

    place_tach_left_batt();
    build_data_panels(parent_);
    live_ready_ = true;
    session_booted_ = true;

    const int batt = static_cast<int>(pros::battery::get_capacity());
    char b[8];
    std::snprintf(b, sizeof(b), "%d", batt);
    set_tach_pct(batt, b);
    if (boot_tag_ != nullptr) {
        lv_label_set_text(boot_tag_, "BATT");
        style_text(boot_tag_, col_muted());
    }
    poke_refresh();
    boot_log::log("telem_dash:live ready");
}

void release() {
    if (parent_ == nullptr) return;
    boot_log::log("telem_dash:release");
    lv_obj_clean(parent_);
    parent_ = nullptr;
    tach_ = tach_val_ = tach_unit_ = boot_tag_ = nullptr;
    brand_ytt_ = brand_type_ = nullptr;
    hdr_auton_ = hdr_ram_ = nullptr;
    pose_x_ = pose_y_ = pose_t_ = path_vals_ = nullptr;
    dist_f_ = dist_b_ = dist_l_ = dist_r_ = nullptr;
    drive_vals_ = mech_vals_ = nullptr;
    cache_auton_[0] = cache_ram_[0] = cache_tach_[0] = '\0';
    cache_px_[0] = cache_py_[0] = cache_pt_[0] = cache_path_[0] = '\0';
    cache_df_[0] = cache_db_[0] = cache_dl_[0] = cache_dr_[0] = '\0';
    cache_drive_[0] = cache_mech_[0] = '\0';
    live_ready_ = false;
}

void focus() {
    if (!ready()) start();
    else poke_refresh();
}

bool ready() { return parent_ != nullptr; }
bool live() { return live_ready_; }
const char* last_error() { return last_error_; }

void refresh() {
    if (parent_ == nullptr || !live_ready_) return;

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
