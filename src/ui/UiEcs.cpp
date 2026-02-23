#include "ui/UiEcs.hpp"

#include <algorithm>
#include <utility>

namespace atm::ui {

Entity UiRegistry::create_entity() {
    const Entity e = next_entity_++;
    entities_.push_back(e);
    return e;
}

void UiRegistry::clear() {
    entities_.clear();
    transforms_.clear();
    panels_.clear();
    labels_.clear();
    buttons_.clear();
    next_entity_ = 1;
}

RectTransformComponent& UiRegistry::add_transform(Entity entity, const sf::FloatRect& rect, int z_index) {
    return transforms_.insert_or_assign(entity, RectTransformComponent{rect, z_index, true}).first->second;
}

PanelComponent& UiRegistry::add_panel(Entity entity, const sf::Color& fill) {
    return panels_.insert_or_assign(entity, PanelComponent{fill}).first->second;
}

LabelComponent& UiRegistry::add_label(
    Entity entity, std::string text, const sf::Vector2f& offset, unsigned int char_size, const sf::Color& color) {
    return labels_.insert_or_assign(entity, LabelComponent{std::move(text), offset, char_size, color}).first->second;
}

ButtonComponent& UiRegistry::add_button(Entity entity, const ButtonComponent& button) {
    return buttons_.insert_or_assign(entity, button).first->second;
}

bool UiRegistry::has_transform(Entity entity) const {
    return transforms_.contains(entity);
}

bool UiRegistry::has_button(Entity entity) const {
    return buttons_.contains(entity);
}

bool UiRegistry::has_label(Entity entity) const {
    return labels_.contains(entity);
}

bool UiRegistry::has_panel(Entity entity) const {
    return panels_.contains(entity);
}

RectTransformComponent* UiRegistry::find_transform(Entity entity) {
    auto it = transforms_.find(entity);
    return it == transforms_.end() ? nullptr : &it->second;
}

const RectTransformComponent* UiRegistry::find_transform(Entity entity) const {
    auto it = transforms_.find(entity);
    return it == transforms_.end() ? nullptr : &it->second;
}

ButtonComponent* UiRegistry::find_button(Entity entity) {
    auto it = buttons_.find(entity);
    return it == buttons_.end() ? nullptr : &it->second;
}

const ButtonComponent* UiRegistry::find_button(Entity entity) const {
    auto it = buttons_.find(entity);
    return it == buttons_.end() ? nullptr : &it->second;
}

LabelComponent* UiRegistry::find_label(Entity entity) {
    auto it = labels_.find(entity);
    return it == labels_.end() ? nullptr : &it->second;
}

const LabelComponent* UiRegistry::find_label(Entity entity) const {
    auto it = labels_.find(entity);
    return it == labels_.end() ? nullptr : &it->second;
}

const PanelComponent* UiRegistry::find_panel(Entity entity) const {
    auto it = panels_.find(entity);
    return it == panels_.end() ? nullptr : &it->second;
}

const std::vector<Entity>& UiRegistry::entities() const {
    return entities_;
}

UiRegistry& UiScene::registry() {
    return registry_;
}

const UiRegistry& UiScene::registry() const {
    return registry_;
}

void UiScene::clear() {
    registry_.clear();
}

bool UiScene::handle_mouse_move(const sf::Vector2f& mouse_px) {
    bool hovered_any = false;
    for (Entity entity : registry_.entities()) {
        auto* button = registry_.find_button(entity);
        const auto* transform = registry_.find_transform(entity);
        if (button == nullptr || transform == nullptr || !transform->visible || !button->enabled) {
            continue;
        }
        button->hovered = hit_test(*transform, mouse_px);
        hovered_any = hovered_any || button->hovered;
    }
    return hovered_any;
}

bool UiScene::handle_left_press(const sf::Vector2f& mouse_px, std::vector<UiEvent>& out_events) {
    struct Hit {
        Entity entity = kInvalidEntity;
        int z_index = -2147483647;
    };

    Hit best;
    for (Entity entity : registry_.entities()) {
        auto* button = registry_.find_button(entity);
        auto* transform = registry_.find_transform(entity);
        if (button == nullptr || transform == nullptr || !transform->visible || !button->enabled) {
            continue;
        }
        if (!hit_test(*transform, mouse_px)) {
            button->pressed = false;
            continue;
        }
        if (transform->z_index >= best.z_index) {
            best = Hit{entity, transform->z_index};
        }
    }

    if (best.entity == kInvalidEntity) {
        return false;
    }

    if (auto* button = registry_.find_button(best.entity)) {
        button->pressed = true;
        out_events.push_back(UiEvent{best.entity, button->action});
    }
    return true;
}

void UiScene::draw(sf::RenderWindow& window, const sf::Font& font, const RetroTheme& theme) const {
    std::vector<Entity> ordered = registry_.entities();
    std::sort(ordered.begin(), ordered.end(), [&](Entity a, Entity b) {
        const auto* ta = registry_.find_transform(a);
        const auto* tb = registry_.find_transform(b);
        const int za = ta != nullptr ? ta->z_index : 0;
        const int zb = tb != nullptr ? tb->z_index : 0;
        return za < zb;
    });

    for (Entity entity : ordered) {
        const auto* transform = registry_.find_transform(entity);
        if (transform == nullptr || !transform->visible) {
            continue;
        }

        if (const auto* panel = registry_.find_panel(entity)) {
            sf::RectangleShape body(transform->rect.size);
            body.setPosition(transform->rect.position);
            body.setFillColor(panel->fill);
            window.draw(body);

            sf::RectangleShape top_edge({transform->rect.size.x, theme.panel_border_thickness});
            top_edge.setPosition(transform->rect.position);
            top_edge.setFillColor(theme.panel_shadow_light);
            window.draw(top_edge);

            sf::RectangleShape left_edge({theme.panel_border_thickness, transform->rect.size.y});
            left_edge.setPosition(transform->rect.position);
            left_edge.setFillColor(theme.panel_shadow_light);
            window.draw(left_edge);

            sf::RectangleShape bottom_edge({transform->rect.size.x, theme.panel_border_thickness});
            bottom_edge.setPosition(
                sf::Vector2f(transform->rect.position.x, transform->rect.position.y + transform->rect.size.y - theme.panel_border_thickness));
            bottom_edge.setFillColor(theme.panel_shadow_dark);
            window.draw(bottom_edge);

            sf::RectangleShape right_edge({theme.panel_border_thickness, transform->rect.size.y});
            right_edge.setPosition(
                sf::Vector2f(transform->rect.position.x + transform->rect.size.x - theme.panel_border_thickness, transform->rect.position.y));
            right_edge.setFillColor(theme.panel_shadow_dark);
            window.draw(right_edge);
        }

        if (const auto* button = registry_.find_button(entity)) {
            sf::RectangleShape body(transform->rect.size);
            body.setPosition(transform->rect.position);
            sf::Color fill = button->fill_normal;
            if (!button->enabled) {
                fill = theme.panel_face;
            } else if (button->pressed) {
                fill = button->fill_pressed;
            } else if (button->hovered) {
                fill = button->fill_hover;
            }
            body.setFillColor(fill);
            window.draw(body);

            const bool inset = button->pressed;
            const sf::Color hi = inset ? theme.panel_shadow_dark : theme.panel_shadow_light;
            const sf::Color lo = inset ? theme.panel_shadow_light : theme.panel_shadow_dark;

            sf::RectangleShape top_edge({transform->rect.size.x, theme.button_border_thickness});
            top_edge.setPosition(transform->rect.position);
            top_edge.setFillColor(hi);
            window.draw(top_edge);

            sf::RectangleShape left_edge({theme.button_border_thickness, transform->rect.size.y});
            left_edge.setPosition(transform->rect.position);
            left_edge.setFillColor(hi);
            window.draw(left_edge);

            sf::RectangleShape bottom_edge({transform->rect.size.x, theme.button_border_thickness});
            bottom_edge.setPosition(sf::Vector2f(
                transform->rect.position.x, transform->rect.position.y + transform->rect.size.y - theme.button_border_thickness));
            bottom_edge.setFillColor(lo);
            window.draw(bottom_edge);

            sf::RectangleShape right_edge({theme.button_border_thickness, transform->rect.size.y});
            right_edge.setPosition(sf::Vector2f(
                transform->rect.position.x + transform->rect.size.x - theme.button_border_thickness, transform->rect.position.y));
            right_edge.setFillColor(lo);
            window.draw(right_edge);

            sf::Text text(font, button->text, 13);
            text.setFillColor(button->enabled ? button->text_color : theme.text_disabled);
            const sf::FloatRect text_bounds = text.getLocalBounds();
            const float tx = transform->rect.position.x + 0.5f * (transform->rect.size.x - text_bounds.size.x);
            const float ty = transform->rect.position.y + 0.5f * (transform->rect.size.y - text_bounds.size.y) - 3.0f;
            text.setPosition({tx + (button->pressed ? 1.0f : 0.0f), ty + (button->pressed ? 1.0f : 0.0f)});
            window.draw(text);
        }

        if (const auto* label = registry_.find_label(entity)) {
            sf::Text text(font, label->text, label->char_size);
            text.setPosition(transform->rect.position + label->offset);
            text.setFillColor(label->color);
            window.draw(text);
        }
    }
}

bool UiScene::hit_test(const RectTransformComponent& rect, const sf::Vector2f& mouse_px) {
    return rect.visible && rect.rect.contains(mouse_px);
}

Entity add_panel(UiRegistry& registry, const sf::FloatRect& rect, int z_index, const sf::Color& fill) {
    const Entity entity = registry.create_entity();
    registry.add_transform(entity, rect, z_index);
    registry.add_panel(entity, fill);
    return entity;
}

Entity add_label(UiRegistry& registry, const sf::FloatRect& rect, int z_index, std::string text, const sf::Vector2f& offset,
    unsigned int char_size, const sf::Color& color) {
    const Entity entity = registry.create_entity();
    registry.add_transform(entity, rect, z_index);
    registry.add_label(entity, std::move(text), offset, char_size, color);
    return entity;
}

Entity add_button(
    UiRegistry& registry, const sf::FloatRect& rect, int z_index, std::string text, std::string action,
    const RetroTheme& theme) {
    const Entity entity = registry.create_entity();
    registry.add_transform(entity, rect, z_index);
    ButtonComponent button;
    button.text = std::move(text);
    button.action = std::move(action);
    button.fill_normal = theme.button_face;
    button.fill_hover = theme.button_face_hover;
    button.fill_pressed = theme.button_face_pressed;
    button.text_color = theme.text_primary;
    registry.add_button(entity, button);
    return entity;
}

} // namespace atm::ui
