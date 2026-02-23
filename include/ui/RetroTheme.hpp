#pragma once

#include <SFML/Graphics/Color.hpp>

namespace atm::ui {

struct RetroTheme {
    sf::Color app_background = sf::Color(18, 24, 32);
    sf::Color panel_face = sf::Color(198, 201, 208);
    sf::Color panel_shadow_dark = sf::Color(58, 64, 74);
    sf::Color panel_shadow_light = sf::Color(244, 246, 250);
    sf::Color button_face = sf::Color(212, 214, 220);
    sf::Color button_face_hover = sf::Color(224, 227, 232);
    sf::Color button_face_pressed = sf::Color(186, 190, 198);
    sf::Color text_primary = sf::Color(20, 24, 30);
    sf::Color text_disabled = sf::Color(108, 112, 120);
    sf::Color accent = sf::Color(44, 96, 124);

    float panel_border_thickness = 1.0f;
    float button_border_thickness = 1.0f;
    float padding = 8.0f;
};

RetroTheme make_default_retro_theme();

} // namespace atm::ui
