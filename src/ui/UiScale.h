#pragma once
// UI scale layer for the ESP32-P4 build (force-included into every project
// C++ file via build_src_flags "-include ui/UiScale.h" in env:viethud_p4).
//
// The whole UI (ui/Dashboard.cpp, ui/Settings.cpp, the status screens in
// main_viethud.cpp) is laid out in the S3's pixel units for a 480x320 /
// 320x480 screen. The P4 board's panel is 800x480, and LVGL now renders at
// that NATIVE resolution (sharp text, full-res map) — so every layout value
// the UI passes to LVGL is scaled x1.5 here, and every coordinate it reads
// back is scaled /1.5, keeping the UI code in its familiar "design units".
// The design screen is 533x320 (gfx->width()/height(); x1.5 = the full 800x480
// panel), so the landscape layout simply gets a wider map column.
// Fonts are swapped for 1.5x versions (src/ui/fonts_p4/ + lv_conf.h).
//
// Code that must address real pixels (display driver, map canvas) calls the
// plain LVGL function through parentheses — `(lv_obj_set_pos)(o, x, y)` —
// which function-like macros never expand.
#if defined(VIETHUD_P4) && defined(__cplusplus)
#include <lvgl.h>

static inline int32_t vhS(int32_t v) {
    if (LV_COORD_IS_SPEC(v) || v >= LV_RADIUS_CIRCLE || v <= -LV_RADIUS_CIRCLE) return v; // LV_PCT/SIZE_CONTENT/CIRCLE
    return v >= 0 ? (v * 3) / 2 : -((-v * 3) / 2);
}
static inline int32_t vhU(int32_t v) { return v >= 0 ? (v * 2 + 1) / 3 : -((-v * 2 + 1) / 3); }

// --- geometry setters ---
#define lv_obj_set_pos(o, x, y) (lv_obj_set_pos)((o), vhS(x), vhS(y))
#define lv_obj_set_size(o, w, h) (lv_obj_set_size)((o), vhS(w), vhS(h))
#define lv_obj_set_width(o, w) (lv_obj_set_width)((o), vhS(w))
#define lv_obj_set_height(o, h) (lv_obj_set_height)((o), vhS(h))
#define lv_obj_set_x(o, x) (lv_obj_set_x)((o), vhS(x))
#define lv_obj_set_y(o, y) (lv_obj_set_y)((o), vhS(y))
#define lv_obj_align(o, a, x, y) (lv_obj_align)((o), (a), vhS(x), vhS(y))
#define lv_obj_align_to(o, b, a, x, y) (lv_obj_align_to)((o), (b), (a), vhS(x), vhS(y))
// --- pixel-valued style properties ---
#define VH_STYLE1(name) \
    static inline void vh_##name(lv_obj_t *o, int32_t v, lv_style_selector_t s) { (lv_obj_set_style_##name)(o, vhS(v), s); }
VH_STYLE1(border_width)
VH_STYLE1(radius)
VH_STYLE1(pad_all)
VH_STYLE1(pad_hor)
VH_STYLE1(pad_ver)
VH_STYLE1(pad_top)
VH_STYLE1(pad_bottom)
VH_STYLE1(pad_left)
VH_STYLE1(pad_right)
VH_STYLE1(pad_row)
VH_STYLE1(pad_column)
VH_STYLE1(pad_gap)
VH_STYLE1(shadow_width)
VH_STYLE1(shadow_offset_x)
VH_STYLE1(shadow_offset_y)
VH_STYLE1(shadow_spread)
VH_STYLE1(outline_width)
VH_STYLE1(outline_pad)
VH_STYLE1(line_width)
VH_STYLE1(arc_width)
VH_STYLE1(text_line_space)
VH_STYLE1(text_letter_space)
VH_STYLE1(translate_x)
VH_STYLE1(translate_y)
#undef VH_STYLE1
#define lv_obj_set_style_border_width vh_border_width
#define lv_obj_set_style_radius vh_radius
#define lv_obj_set_style_pad_all vh_pad_all
#define lv_obj_set_style_pad_hor vh_pad_hor
#define lv_obj_set_style_pad_ver vh_pad_ver
#define lv_obj_set_style_pad_top vh_pad_top
#define lv_obj_set_style_pad_bottom vh_pad_bottom
#define lv_obj_set_style_pad_left vh_pad_left
#define lv_obj_set_style_pad_right vh_pad_right
#define lv_obj_set_style_pad_row vh_pad_row
#define lv_obj_set_style_pad_column vh_pad_column
#define lv_obj_set_style_pad_gap vh_pad_gap
#define lv_obj_set_style_shadow_width vh_shadow_width
#define lv_obj_set_style_shadow_offset_x vh_shadow_offset_x
#define lv_obj_set_style_shadow_offset_y vh_shadow_offset_y
#define lv_obj_set_style_shadow_spread vh_shadow_spread
#define lv_obj_set_style_outline_width vh_outline_width
#define lv_obj_set_style_outline_pad vh_outline_pad
#define lv_obj_set_style_line_width vh_line_width
#define lv_obj_set_style_arc_width vh_arc_width
#define lv_obj_set_style_text_line_space vh_text_line_space
#define lv_obj_set_style_text_letter_space vh_text_letter_space
#define lv_obj_set_style_translate_x vh_translate_x
#define lv_obj_set_style_translate_y vh_translate_y

// --- getters: back to design units ---
#define lv_obj_get_width(o) vhU((lv_obj_get_width)(o))
#define lv_obj_get_height(o) vhU((lv_obj_get_height)(o))
#define lv_obj_get_content_width(o) vhU((lv_obj_get_content_width)(o))
#define lv_obj_get_content_height(o) vhU((lv_obj_get_content_height)(o))
#define lv_obj_get_x(o) vhU((lv_obj_get_x)(o))
#define lv_obj_get_y(o) vhU((lv_obj_get_y)(o))
static inline void vh_indev_get_point(const lv_indev_t *i, lv_point_t *p) {
    (lv_indev_get_point)(i, p);
    p->x = vhU(p->x);
    p->y = vhU(p->y);
}
#define lv_indev_get_point vh_indev_get_point

// --- lines: lv_line keeps the POINTER, so keep a scaled copy alive per source array ---
static inline void vh_line_set_points(lv_obj_t *o, const lv_point_precise_t *pts, uint32_t n) {
    struct Ent { const lv_point_precise_t *src; lv_point_precise_t *dst; uint32_t cap; };
    static Ent ents[16];
    Ent *e = nullptr;
    for (auto &x : ents)
        if (x.src == pts) { e = &x; break; }
    if (!e)
        for (auto &x : ents)
            if (!x.src) { e = &x; e->src = pts; break; }
    if (!e) { (lv_line_set_points)(o, pts, n); return; } // table full: unscaled rather than crash
    if (e->cap < n) {
        e->dst = (lv_point_precise_t *)lv_realloc(e->dst, n * sizeof(lv_point_precise_t));
        e->cap = n;
    }
    for (uint32_t i = 0; i < n; i++) {
        e->dst[i].x = pts[i].x * 3 / 2;
        e->dst[i].y = pts[i].y * 3 / 2;
    }
    (lv_line_set_points)(o, e->dst, n);
}
#define lv_line_set_points vh_line_set_points

// --- images: bitmaps (icons, logo) are drawn 1.5x (smooth); symbol-text sources are fonts, already scaled ---
static inline void vh_image_set_src(lv_obj_t *o, const void *src) {
    (lv_image_set_src)(o, src);
    if (src && lv_image_src_get_type(src) == LV_IMAGE_SRC_VARIABLE) {
        (lv_image_set_scale)(o, 384);
        lv_image_set_antialias(o, true);
        lv_image_set_inner_align(o, LV_IMAGE_ALIGN_CENTER);
    }
}
#define lv_image_set_src vh_image_set_src
#define lv_image_set_scale(o, s) (lv_image_set_scale)((o), (uint32_t)(s) * 3 / 2)
#define lv_image_set_pivot(o, x, y) (lv_image_set_pivot)((o), vhS(x), vhS(y))

// --- fonts: 1.5x counterparts ---
#define lv_font_montserrat_14 lv_font_montserrat_22
#define lv_font_montserrat_20 lv_font_montserrat_30
#define lv_font_montserrat_24 lv_font_montserrat_36
#define lv_font_montserrat_28 lv_font_montserrat_42
#define lv_font_montserrat_32 lv_font_montserrat_48
#define lv_font_montserrat_36 lv_font_montserrat_54
#define lv_font_montserrat_48 lv_font_montserrat_72
#define lv_font_montserrat_speed lv_font_montserrat_speed_x15
#define lv_font_vn_14 lv_font_vn_21
#define lv_font_vn_20 lv_font_vn_30
LV_FONT_DECLARE(lv_font_montserrat_54)
LV_FONT_DECLARE(lv_font_montserrat_72)
LV_FONT_DECLARE(lv_font_vn_30)
#endif
