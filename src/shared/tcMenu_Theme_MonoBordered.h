/*  Helioduino: Simple automation controller for solar tracking systems.
    Copyright (C) 2023 NachtRaveVL          <nachtravevl@gmail.com>
    tcMenu Mono Bordered Theme
*/

#include <Helioduino.h>
#ifdef HELIO_USE_GUI

/*
 * Copyright (c) 2018 https://www.thecoderscorner.com (Dave Cherry).
 * This product is licensed under an Apache license, see the LICENSE file in the top-level directory.
 */

/* Changelist:
 * - Variable name standardizations and keeping things in function scope.
 * - Enclosed inside of #ifdef & reorg'ed for general inclusion
 */

#ifndef TCMENU_THEME_MONO_BORDER
#define TCMENU_THEME_MONO_BORDER

#define TITLE_BORDER_THICKNESS 2
#define TITLE_SPACING 2

inline void installMonoBorderedTheme(GraphicsDeviceRenderer& bgr, const MenuFontDef& itemFont, const MenuFontDef& titleFont, bool needEditingIcons) {
    color_t defaultItemPaletteMono[] = {WHITE, BLACK, WHITE, WHITE};

    bgr.setDisplayDimensions(bgr.getDeviceDrawable()->getDisplayDimensions().x, bgr.getDeviceDrawable()->getDisplayDimensions().y);
    auto& factory = bgr.getGraphicsPropertiesFactory();

    factory.setSelectedColors(0, 1);

    MenuPadding titlePadding(1);
    MenuPadding itemPadding(1);
    int titleHeight = bgr.heightForFontPadding(titleFont.fontData, titleFont.fontMag, titlePadding);
    int itemHeight = bgr.heightForFontPadding(itemFont.fontData, itemFont.fontMag, itemPadding);

    factory.addImageToCache(DrawableIcon(SPECIAL_ID_EDIT_ICON, Coord(8, 6),DrawableIcon::ICON_XBITMAP, loResEditingIcon));
    factory.addImageToCache(DrawableIcon(SPECIAL_ID_ACTIVE_ICON, Coord(8, 6),DrawableIcon::ICON_XBITMAP, loResActiveIcon));

    factory.setDrawingPropertiesDefault(ItemDisplayProperties::COMPTYPE_TITLE, defaultItemPaletteMono, titlePadding, titleFont.fontData, titleFont.fontMag,
                                        TITLE_SPACING, titleHeight + 1, GridPosition::JUSTIFY_TITLE_LEFT_WITH_VALUE,
                                        MenuBorder(0, 0, TITLE_BORDER_THICKNESS, 0));
    factory.setDrawingPropertiesDefault(ItemDisplayProperties::COMPTYPE_ITEM, defaultItemPaletteMono, itemPadding, itemFont.fontData, itemFont.fontMag,
                                        1, itemHeight, GridPosition::JUSTIFY_TITLE_LEFT_VALUE_RIGHT , MenuBorder(0));
    factory.setDrawingPropertiesDefault(ItemDisplayProperties::COMPTYPE_ACTION, defaultItemPaletteMono, itemPadding, itemFont.fontData, itemFont.fontMag,
                                        1, itemHeight, GridPosition::JUSTIFY_TITLE_LEFT_WITH_VALUE, MenuBorder(0));

    tcgfx::ConfigurableItemDisplayPropertiesFactory::refreshCache();
}

#endif //TCMENU_THEME_MONO_BORDER
#endif
