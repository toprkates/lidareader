#include <iostream>
#include <SFML/Graphics.hpp>
#include <SFML/System.hpp>
#include <SFML/Audio.hpp>
#include <SFML/Network.hpp>

#include "file_read.h"
#include "screen.h"
#include <SFML/Graphics.hpp>
#include <vector>
#include <string>

int draw() {
    sf::RenderWindow window(sf::VideoMode({900, 700}), "Robot Visualization");

    // Coordinate scale (to make distances visible)
    const float scale = 100.f;

    // Robot position (origin)
    sf::Vector2f robotPos(0.f, 0.f);

    // Cluster positions
    std::vector<std::pair<std::string, sf::Vector2f>> clusters = {
        {"d1", {1.f, 0.5f}},
        {"d2", {1.1f, 1.2f}},
        {"d3", {1.2f, 1.3f}},
        {"d4", {-1.5f, 2.f}},
        {"d5", {-2.f, 0.f}},
        {"d6", {-1.f, -2.f}}
    };

    // Load font
    sf::Font font;
    if (!font.openFromFile("visuals/arial.ttf")) {
        return -1;
    }

    while (window.isOpen()) {
        while (auto event = window.pollEvent()) {
            if (event->is<sf::Event::Closed>())
                window.close();

            // Example: keyboard event
            if (auto key = event->getIf<sf::Event::KeyPressed>()) {
                if (key->code == sf::Keyboard::Key::Escape)
                    window.close();
            }
        }


        window.clear(sf::Color::White);

        // Draw grid-like background dots
        sf::CircleShape dot(1.f);
        dot.setFillColor(sf::Color(150, 200, 255, 60));
        for (float x = -3; x <= 3; x += 0.1f)
            for (float y = -3; y <= 3; y += 0.1f) {
                dot.setPosition({450 + x * scale, 350 - y * scale});
                window.draw(dot);
            }

        // Draw robot (red circle)
        sf::CircleShape robot(6.f);
        robot.setFillColor(sf::Color::Red);
        robot.setOrigin({6, 6});
        robot.setPosition({450 + robotPos.x * scale, 350 - robotPos.y * scale});
        window.draw(robot);

        // Draw clusters (green)
        for (auto& [name, pos] : clusters) {
            sf::CircleShape cluster(5.f);
            cluster.setFillColor(sf::Color(0, 100, 0));
            cluster.setOrigin({5, 5});
            cluster.setPosition({450 + pos.x * scale, 350 - pos.y * scale});
            window.draw(cluster);

            // Add label
            sf::Text label(font, name, 14);
            label.setFillColor(sf::Color::Black);
            label.setPosition({450 + pos.x * scale + 8, 350 - pos.y * scale - 8});
            window.draw(label);
        }

        // Draw distance line (red dashed)
        sf::Vector2f target = clusters[1].second; // d2
        sf::Vector2f startPos(450 + robotPos.x * scale, 350 - robotPos.y * scale);
        sf::Vector2f endPos(450 + target.x * scale, 350 - target.y * scale);

        // dashed line segments
        int dashCount = 15;
        sf::VertexArray dashed(sf::PrimitiveType::Lines);

        for (int i = 0; i < dashCount; ++i) {
            float t1 = static_cast<float>(i) / dashCount;
            float t2 = static_cast<float>(i + 0.5f) / dashCount;

            sf::Vector2f p1 = startPos + (endPos - startPos) * t1;
            sf::Vector2f p2 = startPos + (endPos - startPos) * t2;

            dashed.append(sf::Vertex({p1, sf::Color::Red}));
            dashed.append(sf::Vertex({p2, sf::Color::Red}));
        }

        window.draw(dashed);


        // Add distance label
        sf::Text distText(font, "1.90m", 16);
        distText.setFillColor(sf::Color::Red);
        distText.setStyle(sf::Text::Bold);
        distText.setPosition({(startPos.x + endPos.x) / 2, (startPos.y + endPos.y) / 2});
        window.draw(distText);

        // Simple legend
        sf::Text legend(font, "Legend:\n🔴 Robot\n🟢 Cluster\n-- Distance", 14);
        legend.setFillColor(sf::Color::Black);
        legend.setPosition({740, 50});
        window.draw(legend);

        window.display();
    }

    return 0;
}