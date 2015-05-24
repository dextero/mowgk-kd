#include <fenv.h>
#include <iostream>
#include <string>

#include <sandbox/utils/types.h>
#include <sandbox/utils/timer.h>
#include <sandbox/window/window.h>
#include <sandbox/rendering/lineStrip.h>

#include "kd_tree.h"

struct ScopedTimer {
    std::string msg;
    struct timespec start;

    ScopedTimer(const std::string& msg):
        msg(msg)
    {
        fprintf(stderr, "%s: start\n", msg.c_str());
        clock_gettime(CLOCK_REALTIME, &start);
    }

    ~ScopedTimer()
    {
        struct timespec end;
        clock_gettime(CLOCK_REALTIME, &end);

        long diff_ns = (end.tv_sec - start.tv_sec) * 10e9 + (end.tv_nsec - start.tv_nsec);
        double diff_s = (double)diff_ns / 10e9;

        fprintf(stderr, "%s: %lfs\n", msg.c_str(), diff_s);
    }
};

class Accumulator
{
    const float mBase;
    const float mStep;
    float mAccumulator;
    bool mRunning;

public:
    Accumulator(float base,
                float step = 1.f):
        mBase(base),
        mStep(step),
        mAccumulator(0.f),
        mRunning(true)
    {}

    void reset()
    {
        mRunning = true;
        mAccumulator = mBase;
    }

    void update()
    {
        if (mRunning) {
            mAccumulator += mStep;
            assert(mAccumulator >= 0.0f);
        }
    }
    void update(float dt)
    {
        if (mRunning) {
            mAccumulator += dt;
            assert(mAccumulator >= 0.0f);
        }
    }

    void stop() { mRunning = false; }
    float getValue() const { return mAccumulator; }
    bool running() const { return mRunning; }
};

static const std::vector<sb::Vec3> WIREFRAME_BOX_VERTICES {
    { -0.5, -0.5, -0.5 },
    { 0.5, -0.5, -0.5 },
    { 0.5, 0.5, -0.5 },
    { -0.5, 0.5, -0.5 },
    { -0.5, -0.5, -0.5 },
    { -0.5, -0.5, 0.5 },
    { -0.5, 0.5, 0.5 },
    { -0.5, 0.5, -0.5 },
    { 0.5, 0.5, -0.5 },
    { 0.5, 0.5, 0.5 },
    { 0.5, -0.5, 0.5 },
    { 0.5, -0.5, -0.5 },
    { 0.5, -0.5, 0.5 },
    { -0.5, -0.5, 0.5 },
    { -0.5, 0.5, 0.5 },
    { 0.5, 0.5, 0.5 },
};

class TreeVisualizer
{
public:
    TreeVisualizer():
        wnd(1440, 900),
        fpsDeltaTime(0.0f, 0.0f),
        colorShader(gResourceMgr.getShader("proj_basic.vert", "color.frag")),
        wireframeBox(WIREFRAME_BOX_VERTICES, sb::Color::White, colorShader)
    {
        wnd.lockCursor();
        wnd.hideCursor();

        wnd.getRenderer().enableFeature(sb::Renderer::Feature::AlphaBlending);
        wnd.getRenderer().enableFeature(sb::Renderer::Feature::DepthTest, false);

        wnd.getCamera().lookAt(sb::Vec3(5.f, 5.f, 20.f), sb::Vec3(5.f, 5.f, 0.f));
    }

    void show(const kd_tree<double> &tree)
    {
        sb::Timer clock;

        while (wnd.isOpened()) {
            float delta = clock.getSecondsElapsed();
            clock.reset();

            handleInput();
            update(delta);
            draw(tree);
        }
    }

private:
    static constexpr float SPEED = 0.5f;
    static constexpr float PHYSICS_UPDATE_STEP = 0.03f;
    static constexpr float fpsUpdateStep = 1.f;   // update FPS-string every fpsUpdateStep seconds

    sb::Window wnd;
    sb::Vec3 speed;

    float fpsCounter;      // how many frames have been rendered in fpsUpdateStep time?
    float fpsCurrValue;    // current FPS value
    Accumulator fpsDeltaTime;
    std::string fpsString = "FPS = ???";

    std::shared_ptr<sb::Shader> colorShader;
    sb::LineStrip wireframeBox;

    size_t highlightLevel = 0;

    void handleMouseMoved(const sb::Event& e)
    {
        if (wnd.hasFocus())
        {
            sb::Vec2i halfSize = wnd.getSize() / 2;
            int pixelsDtX = (int)e.data.mouse.x - halfSize.x;
            int pixelsDtY = (int)e.data.mouse.y - halfSize.y;

            static const float ROTATION_SPEED = 1.0f;
            sb::Radians dtX = sb::Radians(ROTATION_SPEED * (float)pixelsDtX / halfSize.x);
            sb::Radians dtY = sb::Radians(ROTATION_SPEED * (float)pixelsDtY / halfSize.y);

            wnd.getCamera().mouseLook(dtX, dtY);
        }
    }

    void handleInput()
    {
        sb::Event e;
        while (wnd.getEvent(e))
        {
            switch (e.type)
            {
            case sb::Event::MouseMoved:
                handleMouseMoved(e);
                break;
            case sb::Event::WindowFocus:
                if (e.data.focus)
                {
                    wnd.hideCursor();
                    wnd.lockCursor();
                }
                else
                {
                    wnd.hideCursor(false);
                    wnd.lockCursor(false);
                    speed = sb::Vec3(0.0f, 0.0f, 0.0f);
                }
                break;
            case sb::Event::KeyPressed:
                switch (e.data.key) {
                case sb::Key::A: speed.x = -SPEED; break;
                case sb::Key::D: speed.x = SPEED; break;
                case sb::Key::S: speed.z = -SPEED; break;
                case sb::Key::W: speed.z = SPEED; break;
                case sb::Key::Q: speed.y = SPEED; break;
                case sb::Key::Z: speed.y = -SPEED; break;
                case sb::Key::NumpadAdd:
                    ++highlightLevel;
                    break;
                case sb::Key::NumpadSubtract:
                    if (highlightLevel > 0) {
                        --highlightLevel;
                    }
                    break;
                case sb::Key::Esc:
                    wnd.close();
                    break;
                default:
                    break;
                }
                break;
        case sb::Event::KeyReleased:
                switch (e.data.key) {
                case sb::Key::A:
                case sb::Key::D:
                    speed.x = 0.f;
                    break;
                case sb::Key::S:
                case sb::Key::W:
                    speed.z = 0.f;
                    break;
                case sb::Key::Q:
                case sb::Key::Z:
                    speed.y = 0.f;
                    break;
                default:
                    break;
                }
                break;
            case sb::Event::WindowClosed:
                wnd.close();
                break;
            default:
                break;
            }
        }
    }

    void update(float deltaSeconds)
    {
        fpsDeltaTime.update(deltaSeconds);
        ++fpsCounter;

        // FPS update
        if (fpsDeltaTime.getValue() >= fpsUpdateStep)
        {
            fpsCurrValue = fpsCounter / fpsUpdateStep;
            fpsString = "FPS = " + std::to_string(fpsCurrValue);
            fpsDeltaTime.update(-fpsUpdateStep);
            fpsCounter = 0.f;
        }

        wnd.getCamera().moveRelative(speed);
    }

    void drawTree(const kd_tree<double> &tree,
                  const sb::Color &color = sb::Color::White,
                  size_t level = 0)
    {
        if (!tree.is_leaf) {
            drawTree(*tree.data.node.low, sb::Color::Red, level + 1);
            drawTree(*tree.data.node.high, sb::Color::Green, level + 1);
        }

        auto center = tree.bounding_box.center();
        auto scale = tree.bounding_box.size();

        wireframeBox.setPosition(center.x(), center.y(), center.z());
        wireframeBox.setScale(scale.x(), scale.y(), scale.z());
        wireframeBox.setColor({ color.r, color.g, color.b,
                                highlightLevel == level ? 1.0f : 0.05f });

        wnd.draw(wireframeBox);
    }

    void draw(const kd_tree<double> &tree)
    {
        wnd.clear(sb::Color(0.f, 0.f, 0.5f));

        drawTree(tree);

        size_t currLineIdx = 0;
        std::cout << fpsString << "\n";
        wnd.drawString(fpsString, { 0.0f, 0.0f },
                       (fpsCurrValue > 30.f
                           ? sb::Color::Green
                           : (fpsCurrValue > 20.f ? sb::Color::Yellow
                                                  : sb::Color::Red)),
                       currLineIdx++);
        wnd.drawString("highlightLevel = " + std::to_string(highlightLevel),
                       { 0.0f, 0.0f }, sb::Color::White, currLineIdx++);

        wnd.display();
    }
};

int main(int /*argc*/,
         char* /*argv*/[])
{
    feenableexcept(FE_INVALID | FE_DIVBYZERO | FE_OVERFLOW | FE_UNDERFLOW);

    std::unique_ptr<kd_tree<double>> tree;

    {
        ScopedTimer timer("generating k-d tree");
        tree = kd_tree<double>::build({0,0,0,1,1,1},
                                      [](double x, double y, double z) {
                                          return x + y + z;
                                      },
                                      0.25);
    }

    TreeVisualizer visualizer;
    visualizer.show(*tree.get());

    for (double x = 0.0; x < 1.0; x += 0.1) {
        for (double y = 0.0; y < 1.0; y += 0.1) {
            for (double z = 0.0; z < 1.0; z += 0.1) {
                std::cout << x << " " << y << " " << z << " " << tree->value_at(x, y, z) << "\n";
            }
        }
        std::cout << "\n";
    }

    return 0;
}
