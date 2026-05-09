#include "main.h"
#include "Helix/auton_selector.hpp"

using namespace Helix;

// ── Colors ───────────────────────────────────────────────────────────────────
#define C_BG        lv_color_hex(0x0D0D0D)
#define C_PANEL     lv_color_hex(0x141414)
#define C_CARD      lv_color_hex(0x1A1A1A)
#define C_CARD_SEL  lv_color_hex(0x12121F)
#define C_ACCENT    lv_color_hex(0x0044BB)
#define C_RED       lv_color_hex(0xCC3333)
#define C_BLUE      lv_color_hex(0x3333CC)
#define C_WHITE     lv_color_hex(0xFFFFFF)
#define C_GREY      lv_color_hex(0x666666)
#define C_LGREY     lv_color_hex(0xAAAAAA)
#define C_DIVIDER   lv_color_hex(0x222222)
#define C_GREEN     lv_color_hex(0x44CC88)

// ── Helpers ───────────────────────────────────────────────────────────────────
static lv_obj_t* make_rect(lv_obj_t* parent, int x, int y, int w, int h, lv_color_t color) {
    lv_obj_t* obj = lv_obj_create(parent);
    lv_obj_set_pos(obj, x, y);
    lv_obj_set_size(obj, w, h);
    lv_obj_set_style_bg_color(obj, color, 0);
    lv_obj_set_style_bg_opa(obj, LV_OPA_COVER, 0);
    lv_obj_set_style_border_width(obj, 0, 0);
    lv_obj_set_style_pad_all(obj, 0, 0);
    lv_obj_set_style_radius(obj, 0, 0);
    lv_obj_has_flag(obj, LV_OBJ_FLAG_SCROLLABLE); // ← add this
    return obj;
}

static lv_obj_t* make_label(lv_obj_t* parent, int x, int y, const char* text,
                             lv_color_t color, bool large = false) {
    lv_obj_t* label = lv_label_create(parent);
    lv_label_set_text(label, text);
    lv_obj_set_style_text_color(label, color, 0);
    lv_obj_set_style_text_font(label, large ? &lv_font_montserrat_16 : &lv_font_montserrat_12, 0);
    lv_obj_set_pos(label, x, y);
    // Transparent background — no set_style_bg needed, labels default transparent
    return label;
}

// ── Draw field (left panel) ───────────────────────────────────────────────────
static void draw_field(lv_obj_t* parent, bool highlight_red) {
    // Field background
    make_rect(parent, 8, 8, 184, 184, lv_color_hex(0x1A2A1A));

    // X lines
    lv_obj_t* line1 = lv_line_create(parent);
    static lv_point_precise_t pts1[] = {{8,8},{192,192}};
    lv_line_set_points(line1, pts1, 2);
    lv_obj_set_style_line_color(line1, lv_color_hex(0x334433), 0);
    lv_obj_set_style_line_width(line1, 1, 0);

    lv_obj_t* line2 = lv_line_create(parent);
    static lv_point_precise_t pts2[] = {{192,8},{8,192}};
    lv_line_set_points(line2, pts2, 2);
    lv_obj_set_style_line_color(line2, lv_color_hex(0x334433), 0);
    lv_obj_set_style_line_width(line2, 1, 0);

    // Red zones (left wall + bottom)
    lv_color_t red_zone   = highlight_red  ? lv_color_hex(0x661111) : lv_color_hex(0x220000);
    lv_color_t blue_zone  = !highlight_red ? lv_color_hex(0x111166) : lv_color_hex(0x000022);

    make_rect(parent, 8,   8,  44, 44, red_zone);   // top-left red
    make_rect(parent, 8, 148,  44, 44, red_zone);   // bottom-left red
    make_rect(parent, 148,  8, 44, 44, blue_zone);  // top-right blue
    make_rect(parent, 148, 148, 44, 44, blue_zone); // bottom-right blue

    // Zone borders
    lv_obj_t* r1 = lv_obj_create(parent);
    lv_obj_set_pos(r1, 8, 8); lv_obj_set_size(r1, 44, 44);
    lv_obj_set_style_border_color(r1, C_RED, 0);
    lv_obj_set_style_border_width(r1, 1, 0);
    lv_obj_set_style_bg_opa(r1, LV_OPA_TRANSP, 0);
    lv_obj_set_style_radius(r1, 0, 0);

    lv_obj_t* r2 = lv_obj_create(parent);
    lv_obj_set_pos(r2, 8, 148); lv_obj_set_size(r2, 44, 44);
    lv_obj_set_style_border_color(r2, C_RED, 0);
    lv_obj_set_style_border_width(r2, 1, 0);
    lv_obj_set_style_bg_opa(r2, LV_OPA_TRANSP, 0);
    lv_obj_set_style_radius(r2, 0, 0);

    lv_obj_t* b1 = lv_obj_create(parent);
    lv_obj_set_pos(b1, 148, 8); lv_obj_set_size(b1, 44, 44);
    lv_obj_set_style_border_color(b1, C_BLUE, 0);
    lv_obj_set_style_border_width(b1, 1, 0);
    lv_obj_set_style_bg_opa(b1, LV_OPA_TRANSP, 0);
    lv_obj_set_style_radius(b1, 0, 0);

    lv_obj_t* b2 = lv_obj_create(parent);
    lv_obj_set_pos(b2, 148, 148); lv_obj_set_size(b2, 44, 44);
    lv_obj_set_style_border_color(b2, C_BLUE, 0);
    lv_obj_set_style_border_width(b2, 1, 0);
    lv_obj_set_style_bg_opa(b2, LV_OPA_TRANSP, 0);
    lv_obj_set_style_radius(b2, 0, 0);

    // Label
    make_label(parent, 14, 198, "Override", C_GREY);
}

// ── Constructors ──────────────────────────────────────────────────────────────
AutonSelector::AutonSelector() {
    auton_count = 0;
    auton_page_current = 0;
    current_view = SelectorView::LIST;
    screen = nullptr;
    list_panel = nullptr;
    detail_panel = nullptr;
    Autons = {};
}

AutonSelector::AutonSelector(std::vector<Auton> autons) {
    auton_count = autons.size();
    auton_page_current = 0;
    current_view = SelectorView::LIST;
    screen = nullptr;
    list_panel = nullptr;
    detail_panel = nullptr;
    Autons.assign(autons.begin(), autons.end());
}

void AutonSelector::autons_add(std::vector<Auton> autons) {
    auton_count += autons.size();
    auton_page_current = 0;
    Autons.assign(autons.begin(), autons.end());
}

void AutonSelector::selected_auton_call() {
    if (auton_count == 0) return;
    Autons[auton_page_current].auton_call();
}

// ── Init: build the full UI once ──────────────────────────────────────────────
void AutonSelector::init_ui() {
    screen = lv_obj_create(NULL);
    lv_screen_load(screen);
    lv_obj_set_style_bg_color(screen, C_BG, 0);
    lv_obj_set_style_bg_opa(screen, LV_OPA_COVER, 0);
    show_list();
}

// ── LIST VIEW ─────────────────────────────────────────────────────────────────
void AutonSelector::show_list() {
    if (auton_count == 0) return;
    current_view = SelectorView::LIST;

    // Clear screen
    lv_obj_delete(screen);
    screen = lv_obj_create(NULL);
    lv_screen_load(screen);
    lv_obj_set_style_bg_color(screen, C_BG, 0);
    lv_obj_set_style_bg_opa(screen, LV_OPA_COVER, 0);

    // ── Left panel ────────────────────────────────────────────────────────────
    lv_obj_t* left = make_rect(screen, 0, 0, 200, 240, C_PANEL);
    draw_field(left, Autons[auton_page_current].IsRed);

    // ── Divider ───────────────────────────────────────────────────────────────
    make_rect(screen, 200, 0, 2, 240, C_DIVIDER);

    // ── Right panel ───────────────────────────────────────────────────────────
    lv_obj_t* right = make_rect(screen, 202, 0, 278, 240, C_PANEL);

    // Header bar
    lv_obj_t* header = make_rect(right, 0, 0, 278, 30, lv_color_hex(0x161616));
    make_label(header, 10, 8, "SELECT AUTONOMOUS", C_GREY);

    // Page counter badge
    lv_obj_t* badge = make_rect(header, 220, 5, 50, 20, C_ACCENT);
    char pg[16];
    snprintf(pg, sizeof(pg), "%d / %d", auton_page_current + 1, auton_count);
    make_label(badge, 6, 3, pg, C_WHITE);

    // Accent line
    make_rect(right, 0, 30, 278, 2, C_ACCENT);

    // ── Auton cards ───────────────────────────────────────────────────────────
    int scroll_offset = (auton_page_current >= 4) ? auton_page_current - 3 : 0;
    int visible = std::min(4, auton_count);

    for (int i = 0; i < visible && (i + scroll_offset) < auton_count; i++) {
        int idx = i + scroll_offset;
        bool sel = (idx == auton_page_current);
        int cy = 34 + i * 50;

        // Card background
        lv_obj_t* card = make_rect(right, 6, cy, 262, 46, sel ? C_CARD_SEL : C_CARD);
        lv_obj_set_style_border_color(card, sel ? C_ACCENT : C_DIVIDER, 0);
        lv_obj_set_style_border_width(card, 1, 0);
        lv_obj_set_style_radius(card, 3, 0);

        // Color sidebar
        make_rect(card, 0, 0, 4, 46, Autons[idx].IsRed ? C_RED : C_BLUE);

        // Name
        make_label(card, 10, 6, Autons[idx].Name.c_str(),
                   sel ? C_WHITE : C_LGREY, true);

        // Description preview
        std::string preview = Autons[idx].Description.substr(0, 35);
        if (Autons[idx].Description.length() > 35) preview += "...";
        make_label(card, 10, 26, preview.c_str(), C_GREY);

        // Arrow on selected
        if (sel) make_label(card, 248, 12, ">", C_ACCENT, true);

        // Touch callback — capture idx
        lv_obj_add_flag(card, LV_OBJ_FLAG_CLICKABLE);
        lv_obj_set_user_data(card, (void*)(intptr_t)idx);
        lv_obj_add_event_cb(card, [](lv_event_t* e) {
            if (lv_event_get_code(e) == LV_EVENT_CLICKED) {
                AutonSelector* sel_ptr = (AutonSelector*)lv_event_get_user_data(e);
                int tapped = (int)(intptr_t)lv_obj_get_user_data((lv_obj_t*)lv_event_get_target(e));
                sel_ptr->auton_page_current = tapped;
                sel_ptr->show_detail(tapped);
            }
        }, LV_EVENT_CLICKED, this);
    }

    // Scrollbar
    if (auton_count > 4) {
        make_rect(right, 270, 32, 4, 206, lv_color_hex(0x1A1A1A));
        int bar_h = (4 * 206) / auton_count;
        int bar_y = 32 + (scroll_offset * 206) / auton_count;
        make_rect(right, 270, bar_y, 4, bar_h, C_GREY);
    }

    // Bottom accent
    make_rect(right, 0, 236, 278, 4, C_ACCENT);
}

// ── DETAIL VIEW ───────────────────────────────────────────────────────────────
void AutonSelector::show_detail(int index) {
    if (auton_count == 0) return;
    current_view = SelectorView::DETAIL;
    Auton& a = Autons[index];

    // Clear screen
    lv_obj_delete(screen);
    screen = lv_obj_create(NULL);
    lv_screen_load(screen);
    lv_obj_set_style_bg_color(screen, C_BG, 0);
    lv_obj_set_style_bg_opa(screen, LV_OPA_COVER, 0);

    // ── Left panel with highlighted zone ─────────────────────────────────────
    lv_obj_t* left = make_rect(screen, 0, 0, 200, 240, C_PANEL);
    draw_field(left, a.IsRed);

    // ── Divider ───────────────────────────────────────────────────────────────
    make_rect(screen, 200, 0, 2, 240, C_DIVIDER);

    // ── Right panel ───────────────────────────────────────────────────────────
    lv_obj_t* right = make_rect(screen, 202, 0, 278, 240, C_PANEL);

    // Back button
    lv_obj_t* back_btn = make_rect(right, 6, 8, 60, 22, lv_color_hex(0x1A1A1A));
    lv_obj_set_style_border_color(back_btn, C_DIVIDER, 0);
    lv_obj_set_style_border_width(back_btn, 1, 0);
    lv_obj_set_style_radius(back_btn, 3, 0);
    make_label(back_btn, 8, 4, "< back", C_LGREY);
    lv_obj_add_flag(back_btn, LV_OBJ_FLAG_CLICKABLE);
    lv_obj_add_event_cb(back_btn, [](lv_event_t* e) {
        if (lv_event_get_code(e) == LV_EVENT_CLICKED) {
            AutonSelector* s = (AutonSelector*)lv_event_get_user_data(e);
            s->show_list();
        }
    }, LV_EVENT_CLICKED, this);

    // Color badge
    lv_obj_t* badge = make_rect(right, 210, 8, 56, 22, a.IsRed ? C_RED : C_BLUE);
    lv_obj_set_style_radius(badge, 3, 0);
    make_label(badge, 10, 4, a.IsRed ? "RED" : "BLUE", C_WHITE);

    // Auton name
    make_label(right, 6, 40, a.Name.c_str(), C_WHITE, true);

    // Divider under name
    make_rect(right, 6, 62, 266, 1, C_DIVIDER);

    // Description header
    make_label(right, 6, 70, "DESCRIPTION", C_GREY);

    // Description text — split at 40 chars
    std::string desc = a.Description;
    if (desc.length() > 40) {
        make_label(right, 6, 86, desc.substr(0, 40).c_str(), C_LGREY);
        make_label(right, 6, 102, desc.substr(40).c_str(), C_LGREY);
    } else {
        make_label(right, 6, 86, desc.c_str(), C_LGREY);
    }

    // Alliance
    make_label(right, 6, 128, "ALLIANCE", C_GREY);
    make_label(right, 6, 144, a.IsRed ? "Red Alliance" : "Blue Alliance",
               a.IsRed ? C_RED : C_BLUE, true);
               
    // Expected points
    make_label(right, 6, 168, "EXPECTED POINTS", C_GREY);
    lv_obj_t* pts_box = make_rect(right, 6, 182, 80, 28, lv_color_hex(0x1A2A1A));
    lv_obj_set_style_radius(pts_box, 3, 0);
    char pts_str[16];
    snprintf(pts_str, sizeof(pts_str), "+%d pts", a.ExpectedPoints);
    make_label(pts_box, 8, 6, pts_str, C_GREEN);

    // Confirm button
    lv_obj_t* confirm = make_rect(right, 6, 192, 266, 40, C_ACCENT);
    lv_obj_set_style_radius(confirm, 4, 0);
    make_label(confirm, 60, 12, "CONFIRM SELECTION", C_WHITE, true);
    lv_obj_add_flag(confirm, LV_OBJ_FLAG_CLICKABLE);
    lv_obj_add_event_cb(confirm, [](lv_event_t* e) {
        if (lv_event_get_code(e) == LV_EVENT_CLICKED) {
            AutonSelector* s = (AutonSelector*)lv_event_get_user_data(e);
            // Flash green to confirm
            lv_obj_t* btn = (lv_obj_t*)lv_event_get_target(e);
            lv_obj_set_style_bg_color(btn, lv_color_hex(0x44CC88), 0);
            // Find and update label
            lv_obj_t* lbl = (lv_obj_t*)lv_obj_get_child(btn, 0);
            if (lbl) lv_label_set_text(lbl, "CONFIRMED!");
            // Return to list after short delay via task
            pros::Task([](void* param) {
                pros::delay(600);
                ((AutonSelector*)param)->show_list();
                pros::Task::current().remove();
            }, s, "confirm_flash");
        }
    }, LV_EVENT_CLICKED, this);

    // Bottom accent
    make_rect(right, 0, 236, 278, 4, C_ACCENT);
}

// Legacy — keep so existing call in main.cpp still compiles
void Helix::AutonSelector::selected_auton_print() {
    init_ui();
}