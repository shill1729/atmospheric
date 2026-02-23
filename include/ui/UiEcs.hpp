#pragma once

#include "ui/RetroTheme.hpp"

#include <SFML/Graphics.hpp>

#include <cstdint>
#include <string>
#include <unordered_map>
#include <vector>

namespace atm::ui {

using Entity = std::uint32_t;
constexpr Entity kInvalidEntity = 0;

struct RectTransformComponent {
    sf::FloatRect rect;
    int z_index = 0;
    bool visible = true;
};

struct PanelComponent {
    sf::Color fill;
};

struct LabelComponent {
    std::string text;
    sf::Vector2f offset;
    unsigned int char_size = 13;
    sf::Color color;
};

struct ButtonComponent {
    std::string text;
    std::string action;
    sf::Color fill_normal;
    sf::Color fill_hover;
    sf::Color fill_pressed;
    sf::Color text_color;
    bool enabled = true;
    bool hovered = false;
    bool pressed = false;
};

struct UiEvent {
    Entity entity = kInvalidEntity;
    std::string action;
};

class UiRegistry {
public:
    Entity create_entity();
    void clear();

    RectTransformComponent& add_transform(Entity entity, const sf::FloatRect& rect, int z_index = 0);
    PanelComponent& add_panel(Entity entity, const sf::Color& fill);
    LabelComponent& add_label(Entity entity, std::string text, const sf::Vector2f& offset, unsigned int char_size,
        const sf::Color& color);
    ButtonComponent& add_button(Entity entity, const ButtonComponent& button);

    bool has_transform(Entity entity) const;
    bool has_button(Entity entity) const;
    bool has_label(Entity entity) const;
    bool has_panel(Entity entity) const;

    RectTransformComponent* find_transform(Entity entity);
    const RectTransformComponent* find_transform(Entity entity) const;
    ButtonComponent* find_button(Entity entity);
    const ButtonComponent* find_button(Entity entity) const;
    LabelComponent* find_label(Entity entity);
    const LabelComponent* find_label(Entity entity) const;
    const PanelComponent* find_panel(Entity entity) const;

    const std::vector<Entity>& entities() const;

private:
    Entity next_entity_ = 1;
    std::vector<Entity> entities_;
    std::unordered_map<Entity, RectTransformComponent> transforms_;
    std::unordered_map<Entity, PanelComponent> panels_;
    std::unordered_map<Entity, LabelComponent> labels_;
    std::unordered_map<Entity, ButtonComponent> buttons_;
};

class UiScene {
public:
    UiRegistry& registry();
    const UiRegistry& registry() const;

    void clear();
    bool handle_mouse_move(const sf::Vector2f& mouse_px);
    bool handle_left_press(const sf::Vector2f& mouse_px, std::vector<UiEvent>& out_events);
    void draw(sf::RenderWindow& window, const sf::Font& font, const RetroTheme& theme) const;

private:
    static bool hit_test(const RectTransformComponent& rect, const sf::Vector2f& mouse_px);
    UiRegistry registry_;
};

Entity add_panel(UiRegistry& registry, const sf::FloatRect& rect, int z_index, const sf::Color& fill);
Entity add_label(UiRegistry& registry, const sf::FloatRect& rect, int z_index, std::string text, const sf::Vector2f& offset,
    unsigned int char_size, const sf::Color& color);
Entity add_button(
    UiRegistry& registry, const sf::FloatRect& rect, int z_index, std::string text, std::string action,
    const RetroTheme& theme);

} // namespace atm::ui
