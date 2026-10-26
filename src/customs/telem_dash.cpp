#include "customs/telem_dash.hpp"

#include "customs/boot_log.hpp"
#include "customs/dash_hardware_config.hpp"
#include "customs/objDeclare.hpp"
#include "customs/ui_pages.hpp"
#include "main.h"
#include "pros/motors.h"
#include "robodash/api.h"

#include <algorithm>
#include <cmath>
#include <cstdio>
#include <cstring>

namespace telem_dash {
namespace {

inline lv_color_t col_bg() { return lv_color_hex(0x0B0F14); }
inline lv_color_t col_card() { return lv_color_hex(0x151C26); }
inline lv_color_t col_card2() { return lv_color_hex(0x1C2533); }
inline lv_color_t col_accent() { return lv_color_hex(0x3DDC97); }
inline lv_color_t col_text() { return lv_color_hex(0xF0F4F8); }
inline lv_color_t col_muted() { return lv_color_hex(0x7E8B9A); }
inline lv_color_t col_warn() { return lv_color_hex(0xF0A020); }
inline lv_color_t col_hot() { return lv_color_hex(0xE53935); }
inline lv_color_t col_dim() { return lv_color_hex(0x2A3544); }

rd_view_t* view_ = nullptr;

lv_obj_t* hdr_auton_ = nullptr;
lv_obj_t* hdr_batt_ = nullptr;
lv_obj_t* hdr_lvgl_ = nullptr;
lv_obj_t* batt_bar_ = nullptr;

lv_obj_t* pose_x_ = nullptr;
lv_obj_t* pose_y_ = nullptr;
lv_obj_t* pose_t_ = nullptr;
lv_obj_t* path_vals_ = nullptr;

// Compact DIST value labels — only for configured (non-null) sensors.
constexpr std::size_t kMaxDistUi = 4;
struct DistUi {
    pros::Distance* sensor = nullptr;
    lv_obj_t* lab = nullptr;
    char cache[28] = {};
};
DistUi dist_ui_[kMaxDistUi] = {};
std::size_t dist_ui_n_ = 0;

lv_obj_t* drive_vals_ = nullptr;
lv_obj_t* mech_vals_ = nullptr;

char cache_auton_[40] = {};
char cache_batt_[24] = {};
char cache_lvgl_[24] = {};
char cache_px_[24] = {};
char cache_py_[24] = {};
char cache_pt_[24] = {};
char cache_path_[32] = {};
char cache_drive_[72] = {};
char cache_mech_[64] = {};

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
    lv_obj_set_style_radius(card, 10, 0);
    lv_obj_set_style_border_width(card, 0, 0);
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

lv_color_t conf_color(int c) {
    if (c >= 80) return col_accent();
    if (c >= 50) return col_text();
    if (c >= 25) return col_warn();
    return col_hot();
}

void on_pid_btn(lv_event_t* e) {
    (void)e;
    ui_pages::show_pid();
}

std::size_t count_motors(dash_hw::TempGroup group) {
    std::size_t n = 0;
    for (std::size_t i = 0; i < dash_hw::kMotorTempCount; ++i) {
        if (dash_hw::kMotorTemps[i].group == group) ++n;
    }
    return n;
}

bool temp_ok(double t) { return std::isfinite(t) && t >= 0.0 && t < 100.0; }

// One compact label per group (heap-friendly). Drive inserts '|' at left/right midpoint.
void format_motor_temps(char* out, std::size_t out_n, dash_hw::TempGroup group, double& max_t) {
    max_t = 0.0;
    if (out == nullptr || out_n == 0) return;
    out[0] = '\0';

    const std::size_t group_n = count_motors(group);
    const std::size_t mid =
        (group == dash_hw::TempGroup::Drive && group_n > 1) ? (group_n / 2) : 0;

    std::size_t used = 0;
    std::size_t idx = 0;
    for (std::size_t i = 0; i < dash_hw::kMotorTempCount; ++i) {
        const auto& slot = dash_hw::kMotorTemps[i];
        if (slot.group != group || slot.tag == nullptr) continue;

        if (mid > 0 && idx == mid && used + 3 < out_n) {
            used += static_cast<std::size_t>(std::snprintf(out + used, out_n - used, " |"));
        }

        const double t = pros::c::motor_get_temperature(slot.port);
        if (temp_ok(t) && t > max_t) max_t = t;

        const char* sep = (used == 0) ? "" : " ";
        int wrote = 0;
        if (temp_ok(t)) {
            // tag + width-3 temp => "LF 35" / "LF100"
            wrote = std::snprintf(out + used, out_n - used, "%s%s%3.0f", sep, slot.tag, t);
        } else {
            wrote = std::snprintf(out + used, out_n - used, "%s%s --", sep, slot.tag);
        }
        if (wrote < 0) break;
        used += static_cast<std::size_t>(wrote);
        if (used >= out_n) {
            out[out_n - 1] = '\0';
            break;
        }
        ++idx;
    }
    if (idx == 0) {
        std::snprintf(out, out_n, "--");
    }
}

} // namespace

void start() {
    if (view_ != nullptr) return;

    boot_log::log("telem_dash:start begin");
    // Caller (ui_pages) usually already released PID; belt-and-suspenders.
    // Do not include pid_tuner.hpp cycle here — ui_pages::show_dash owns ordering.
    view_ = rd_view_create("Dash");
    if (view_ == nullptr) {
        boot_log::log("telem_dash:start FAIL view");
        return;
    }
    rd_view_set_anims(view_, RD_ANIM_OFF);
    lv_obj_t* parent = rd_view_obj(view_);
    if (parent == nullptr) {
        rd_view_del(view_);
        view_ = nullptr;
        return;
    }

    lv_obj_set_style_bg_color(parent, col_bg(), 0);
    lv_obj_set_style_bg_opa(parent, LV_OPA_COVER, 0);

    // —— Header ——
    make_label(parent, "EzDrive", 10, 4, col_accent(), &lv_font_montserrat_16);
    hdr_auton_ = make_label(parent, "auton --", 100, 6, col_muted(), &lv_font_montserrat_12);

    batt_bar_ = lv_bar_create(parent);
    if (batt_bar_ != nullptr) {
        lv_obj_set_pos(batt_bar_, 280, 8);
        lv_obj_set_size(batt_bar_, 72, 12);
        lv_bar_set_range(batt_bar_, 0, 100);
        lv_bar_set_value(batt_bar_, 0, LV_ANIM_OFF);
        lv_obj_set_style_bg_color(batt_bar_, col_dim(), LV_PART_MAIN);
        lv_obj_set_style_bg_opa(batt_bar_, LV_OPA_COVER, LV_PART_MAIN);
        lv_obj_set_style_bg_color(batt_bar_, col_accent(), LV_PART_INDICATOR);
        lv_obj_set_style_radius(batt_bar_, 4, LV_PART_MAIN);
        lv_obj_set_style_radius(batt_bar_, 4, LV_PART_INDICATOR);
    }
    hdr_batt_ = make_label(parent, "--%", 354, 6, col_muted(), &lv_font_montserrat_12);
    hdr_lvgl_ = make_label(parent, "LV --", 392, 6, col_muted(), &lv_font_montserrat_12);

    // Header row — left of batt bar; clear of robodash chrome (~444).
    lv_obj_t* pid_btn = lv_btn_create(parent);
    if (pid_btn != nullptr) {
        lv_obj_set_pos(pid_btn, 230, 3);
        lv_obj_set_size(pid_btn, 44, 20);
        lv_obj_set_style_bg_color(pid_btn, col_card2(), 0);
        lv_obj_set_style_radius(pid_btn, 6, 0);
        lv_obj_t* plab = lv_label_create(pid_btn);
        if (plab != nullptr) {
            lv_label_set_text(plab, "PID");
            style_font(plab, &lv_font_montserrat_12);
            style_text(plab, col_accent());
            lv_obj_center(plab);
        }
        lv_obj_add_event_cb(pid_btn, on_pid_btn, LV_EVENT_CLICKED, nullptr);
    }

    // —— Pose hero (left) ——
    lv_obj_t* pose_card = make_card(parent, 8, 30, 232, 128, col_card());
    if (pose_card != nullptr) {
        make_label(pose_card, "POSE", 12, 6, col_muted(), &lv_font_montserrat_12);
        make_label(pose_card, "X", 12, 28, col_muted(), &lv_font_montserrat_12);
        make_label(pose_card, "Y", 12, 56, col_muted(), &lv_font_montserrat_12);
        make_label(pose_card, "T", 12, 84, col_muted(), &lv_font_montserrat_12);
        pose_x_ = make_label(pose_card, "---.--", 36, 22, col_text(), &lv_font_montserrat_20);
        pose_y_ = make_label(pose_card, "---.--", 36, 50, col_text(), &lv_font_montserrat_20);
        pose_t_ = make_label(pose_card, "---.-", 36, 78, col_accent(), &lv_font_montserrat_20);
        path_vals_ = make_label(pose_card, "traveled: --.-- in", 12, 106, col_muted(), &lv_font_montserrat_12);
    }

    // —— Dist (right): only create cells for configured sensors ——
    lv_obj_t* dist_card = make_card(parent, 248, 30, 224, 128, col_card());
    dist_ui_n_ = 0;
    if (dist_card != nullptr) {
        make_label(dist_card, "DIST", 12, 6, col_muted(), &lv_font_montserrat_12);

        static constexpr lv_coord_t kCellPos[kMaxDistUi][2] = {
            {10, 28},
            {114, 28},
            {10, 76},
            {114, 76},
        };

        for (std::size_t i = 0; i < dash_hw::kDistSlotCount && dist_ui_n_ < kMaxDistUi; ++i) {
            const auto& slot = dash_hw::kDistSlots[i];
            if (slot.sensor == nullptr || slot.tag == nullptr) continue;

            const lv_coord_t x = kCellPos[dist_ui_n_][0];
            const lv_coord_t y = kCellPos[dist_ui_n_][1];
            lv_obj_t* box = make_card(dist_card, x, y, 100, 44, col_card2());
            if (box == nullptr) continue;
            make_label(box, slot.tag, 8, 4, col_muted(), &lv_font_montserrat_12);
            DistUi& ui = dist_ui_[dist_ui_n_];
            ui.sensor = slot.sensor;
            ui.lab = make_label(box, "--.-", 8, 20, col_text(), &lv_font_montserrat_16);
            ui.cache[0] = '\0';
            ++dist_ui_n_;
        }
    }

    // —— Temps (tags + values from dash_hw::kMotorTemps) ——
    lv_obj_t* temp_card = make_card(parent, 8, 166, 464, 54, col_card());
    if (temp_card != nullptr) {
        make_label(temp_card, "DRIVE °C", 12, 4, col_muted(), &lv_font_montserrat_12);
        drive_vals_ = make_label(temp_card, "--", 12, 24, col_text(), &lv_font_montserrat_12);
        make_label(temp_card, "MECH °C", 280, 4, col_muted(), &lv_font_montserrat_12);
        mech_vals_ = make_label(temp_card, "--", 280, 24, col_text(), &lv_font_montserrat_12);
    }

    boot_log::log("telem_dash:start done");
    boot_log::lvgl("dash_started");
}

void release() {
    if (view_ == nullptr) return;
    boot_log::log("telem_dash:release");
    rd_view_del(view_);
    view_ = nullptr;
    hdr_auton_ = hdr_batt_ = hdr_lvgl_ = batt_bar_ = nullptr;
    pose_x_ = pose_y_ = pose_t_ = path_vals_ = nullptr;
    for (std::size_t i = 0; i < kMaxDistUi; ++i) {
        dist_ui_[i] = DistUi{};
    }
    dist_ui_n_ = 0;
    drive_vals_ = mech_vals_ = nullptr;
    cache_auton_[0] = cache_batt_[0] = cache_lvgl_[0] = '\0';
    cache_px_[0] = cache_py_[0] = cache_pt_[0] = cache_path_[0] = '\0';
    cache_drive_[0] = cache_mech_[0] = '\0';
}

void focus() {
    if (view_ == nullptr) start();
    if (view_ != nullptr) rd_view_focus(view_);
}

bool ready() { return view_ != nullptr; }

void refresh() {
    if (view_ == nullptr) return;

    char buf[80];

    std::string auton = selector.get_auton()
                            .value_or(rd::Selector::routine_t{"No auton", []() {}})
                            .name;
    if (auton.size() > 18) auton.resize(18);
    set_cached(hdr_auton_, cache_auton_, sizeof(cache_auton_), auton.c_str());

    const int batt = static_cast<int>(pros::battery::get_capacity());
    if (batt_bar_ != nullptr) {
        lv_bar_set_value(batt_bar_, batt, LV_ANIM_OFF);
        lv_obj_set_style_bg_color(batt_bar_,
                                  batt < 30 ? col_hot() : (batt < 50 ? col_warn() : col_accent()),
                                  LV_PART_INDICATOR);
    }
    std::snprintf(buf, sizeof(buf), "%d%%", batt);
    set_cached(hdr_batt_, cache_batt_, sizeof(cache_batt_), buf);

    lv_mem_monitor_t mon{};
    lv_mem_monitor(&mon);
    const uint32_t used = mon.total_size - mon.free_size;
    const unsigned pct =
        mon.total_size > 0 ? static_cast<unsigned>((used * 100U) / mon.total_size) : 0;
    std::snprintf(buf, sizeof(buf), "LV%u", pct);
    set_cached(hdr_lvgl_, cache_lvgl_, sizeof(cache_lvgl_), buf);
    if (hdr_lvgl_ != nullptr) {
        style_text(hdr_lvgl_, pct >= 85 ? col_hot() : (pct >= 70 ? col_warn() : col_muted()));
    }

    const auto pose = chassis.getPose();
    std::snprintf(buf, sizeof(buf), "%7.2f", pose.x);
    set_cached(pose_x_, cache_px_, sizeof(cache_px_), buf);
    std::snprintf(buf, sizeof(buf), "%7.2f", pose.y);
    set_cached(pose_y_, cache_py_, sizeof(cache_py_), buf);
    std::snprintf(buf, sizeof(buf), "%6.1f", pose.theta);
    set_cached(pose_t_, cache_pt_, sizeof(cache_pt_), buf);

    std::snprintf(buf, sizeof(buf), "traveled: %5.2f in", ezPID.get_traveled());
    set_cached(path_vals_, cache_path_, sizeof(cache_path_), buf);

    for (std::size_t i = 0; i < dist_ui_n_; ++i) {
        DistUi& ui = dist_ui_[i];
        if (ui.sensor == nullptr || ui.lab == nullptr) continue;
        const int32_t c = ui.sensor->get_confidence();
        const int conf = (c == PROS_ERR || c < 0 || c > 100) ? 0 : static_cast<int>(c);
        const float inches = static_cast<float>(get_kf_dist(*ui.sensor));
        char b[28];
        std::snprintf(b, sizeof(b), "%5.1f  %2d", inches, conf);
        set_cached(ui.lab, ui.cache, sizeof(ui.cache), b);
        style_text(ui.lab, conf_color(conf));
    }

    double drive_max = 0.0;
    format_motor_temps(buf, sizeof(buf), dash_hw::TempGroup::Drive, drive_max);
    set_cached(drive_vals_, cache_drive_, sizeof(cache_drive_), buf);
    if (drive_vals_ != nullptr) style_text(drive_vals_, temp_color(drive_max));

    double mech_max = 0.0;
    format_motor_temps(buf, sizeof(buf), dash_hw::TempGroup::Mech, mech_max);
    set_cached(mech_vals_, cache_mech_, sizeof(cache_mech_), buf);
    if (mech_vals_ != nullptr) style_text(mech_vals_, temp_color(mech_max));
}

} // namespace telem_dash
