/**
 * An example kd-tree visualization program.
 *
 * For an example on how to use the tree class, see main() function
 * at the end of file.
 */
#include <fenv.h>
#include <iostream>
#include <string>
#include <cmath>

#include <sandbox/utils/types.h>
#include <sandbox/utils/timer.h>
#include <sandbox/window/window.h>
#include <sandbox/rendering/lineStrip.h>
#include <sandbox/rendering/model.h>
#include <sandbox/resources/mesh.h>

// HACK: allow kd-tree structure inspection
#define private public
#include "kd_tree.h"
#undef private

/**
 * An auxiliary class for easy execution time measurement.
 * Displays measured time to the standard error stream in its destructor.
 */
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

static const std::vector<sb::Vec3> BOX_VERTICES {
    { 0.5, 0.5, 0.5 },
    { 0.5, 0.5, -0.5 },
    { 0.5, -0.5, 0.5 },
    { 0.5, -0.5, -0.5 },
    { -0.5, 0.5, 0.5 },
    { -0.5, 0.5, -0.5 },
    { -0.5, -0.5, 0.5 },
    { -0.5, -0.5, -0.5 },
};

#define H 0.3f
#define L 0.1f
#define M ((H + L) / 2.0f)
static const std::vector<sb::Color> BOX_COLORS {
    { M, L, L },
    { L, M, L },
    { H, L, L },
    { L, H, L },
    { L, L, M },
    { M, M, L },
    { L, L, H },
    { H, H, L },
};
#undef M
#undef H
#undef L

static const std::vector<sb::Color> BOX_WHITE_COLORS {
    { 1.0f, 1.0f, 1.0f },
    { 1.0f, 1.0f, 1.0f },
    { 1.0f, 1.0f, 1.0f },
    { 1.0f, 1.0f, 1.0f },
    { 1.0f, 1.0f, 1.0f },
    { 1.0f, 1.0f, 1.0f },
    { 1.0f, 1.0f, 1.0f },
    { 1.0f, 1.0f, 1.0f },
};

static const std::vector<uint32_t> BOX_INDICES {
    0, 1, 2, 1, 2, 3,
    1, 5, 3, 5, 3, 7,
    5, 4, 7, 4, 7, 6,
    4, 0, 2, 0, 2, 6,
    0, 4, 1, 4, 1, 5,
    3, 7, 2, 7, 2, 6,
};

static const std::vector<uint32_t> BOX_OUTSIDE_INDICES {
    0, 2, 1, 2, 1, 3,
    1, 3, 5, 3, 5, 7,
    5, 7, 4, 7, 4, 6,
    4, 2, 0, 2, 0, 6,
    0, 1, 4, 1, 4, 5,
    3, 2, 7, 2, 7, 6,
};

enum class KdChild
{
    Invalid,
    Low,
    High
};

class KdTreePath
{
public:
    inline bool empty() const
    {
        return path.empty();
    }

    inline size_t size() const
    {
        return path.size();
    }

    inline void push(KdChild child)
    {
        assert(child != KdChild::Invalid);

        path.push_back(child == KdChild::High);
    }

    inline void pop()
    {
        path.pop_back();
    }

    inline KdChild operator[](size_t idx) const
    {
        if (idx >= path.size()) {
            return KdChild::Invalid;
        }
        return path[idx] ? KdChild::High : KdChild::Low;
    }

    std::string toString() const
    {
        std::stringstream ss;
        for (bool b: path) {
            ss << (b ? 'H' : 'L');
        }
        return ss.str();
    }

    const kd_tree<double> *apply(const kd_tree<double> &tree)
    {
        const kd_tree<double> *ret = &tree;
        size_t idx = 0;

        while (!ret->is_leaf && idx < path.size()) {
            ret = path[idx] ? ret->data.node.high.get()
                            : ret->data.node.low.get();
            ++idx;
        }

        if (idx >= path.size()) {
            return nullptr;
        }
        return ret;
    }

private:
    std::vector<bool> path;
};

class TreeVisualizer
{
public:
    TreeVisualizer():
        wnd(1920, 1020),
        fpsDeltaTime(0.0f, 0.0f),
        colorShader(gResourceMgr.getShader("proj_color.vert", "color.frag")),
        wireframeBox(WIREFRAME_BOX_VERTICES, sb::Color::White, colorShader),
        boxMesh(std::make_shared<sb::Mesh>(sb::Mesh::Shape::TriangleStrip,
                                           BOX_VERTICES,
                                           std::vector<sb::Vec2>(),
                                           BOX_COLORS,
                                           std::vector<sb::Vec3>(),
                                           BOX_INDICES,
                                           nullptr)),
        solidBoxMesh(std::make_shared<sb::Mesh>(sb::Mesh::Shape::TriangleStrip,
                                                BOX_VERTICES,
                                                std::vector<sb::Vec2>(),
                                                BOX_WHITE_COLORS,
                                                std::vector<sb::Vec3>(),
                                                BOX_OUTSIDE_INDICES,
                                                nullptr)),
        solidBox(solidBoxMesh, colorShader),
        backgroundBox(boxMesh, colorShader)
    {
        wnd.lockCursor();
        wnd.hideCursor();

        wnd.getRenderer().enableFeature(sb::Renderer::Feature::AlphaBlending);
        wnd.getRenderer().enableFeature(sb::Renderer::Feature::DepthTest, false);

        wnd.getCamera().lookAt({-3.f, 2.f, -3.f}, {0.f, 0.f, 0.f});

        backgroundBox.setScale(1000.f);
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
    bool drawSelection = true;

    float fpsCounter;      // how many frames have been rendered in fpsUpdateStep time?
    float fpsCurrValue;    // current FPS value
    Accumulator fpsDeltaTime;
    std::string fpsString = "FPS = ???";

    std::shared_ptr<sb::Shader> colorShader;
    sb::LineStrip wireframeBox;
    std::shared_ptr<sb::Mesh> boxMesh;
    std::shared_ptr<sb::Mesh> solidBoxMesh;
    sb::Model solidBox;
    sb::Model backgroundBox;

    KdTreePath selectedPath;

    void handleMouseMoved(const sb::Event& e)
    {
        if (wnd.hasFocus())
        {
            sb::Vec2i halfSize = wnd.getSize() / 2;
            int pixelsDtX = (int)e.data.mouse.x - halfSize.x;
            int pixelsDtY = (int)e.data.mouse.y - halfSize.y;

            static const float ROTATION_SPEED = 0.3f;
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
                case sb::Key::Numpad0:
                    if (!selectedPath.empty()) {
                        selectedPath.pop();
                    }
                    break;
                case sb::Key::Numpad1:
                    selectedPath.push(KdChild::Low);
                    break;
                case sb::Key::Numpad2:
                    selectedPath.push(KdChild::High);
                    break;
                case sb::Key::Space:
                    drawSelection = !drawSelection;
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
        backgroundBox.setPosition(wnd.getCamera().getEye());
    }

    size_t boxesDrawn = 0;

    sb::Color valueToColor(double value) {
        double hue = sb::math::clamp(value * 255.0, 0.0, 255.0);
        float red, grn, blu;
        float i, f, p, q, t;

        hue/=60;
        i = floor(hue);
        f = hue-i;
        p = 0;
        q = 1*(1-f);
        t = f;
        if (i==0) {red=1; grn=t; blu=p;}
        else if (i==1) {red=q; grn=1; blu=p;}
        else if (i==2) {red=p; grn=1; blu=t;}
        else if (i==3) {red=p; grn=q; blu=1;}
        else if (i==4) {red=t; grn=p; blu=1;}
        else {red=1; grn=p; blu=q;}

        return { red, grn, blu };
    }

    void drawTreeSelection(const kd_tree<double> &tree,
                           sb::Color color = sb::Color::White,
                           size_t level = 0,
                           bool match = true)
    {
        if (level < selectedPath.size() && match && !tree.is_leaf) {
            KdChild hlChild = selectedPath[level];

            drawTreeSelection(*tree.data.node.low,  sb::Color::Red,   level + 1, match && hlChild == KdChild::Low);
            drawTreeSelection(*tree.data.node.high, sb::Color::Green, level + 1, match && hlChild == KdChild::High);
        }

        auto center = tree.bounding_box.center();
        auto scale = tree.bounding_box.size();

        wireframeBox.setPosition(center.x(), center.y(), center.z());
        wireframeBox.setScale(scale.x(), scale.y(), scale.z());

        if (level == selectedPath.size()) {
            if (match) {
                color = sb::Color::White;
            }
        } else {
            color.a = 0.1f;
        }
        wireframeBox.setColor(color);

        wnd.draw(wireframeBox);
        ++boxesDrawn;
    }

    void drawTree(const kd_tree<double> &tree,
                  size_t level = 0,
                  bool match = true)
    {
        if (level < selectedPath.size() /*&& match*/ && !tree.is_leaf) {
            KdChild hlChild = selectedPath[level];

            drawTree(*tree.data.node.low,  level + 1, match && hlChild == KdChild::Low);
            drawTree(*tree.data.node.high, level + 1, match && hlChild == KdChild::High);
        }

        auto center = tree.bounding_box.center();
        auto scale = tree.bounding_box.size();

        sb::Drawable *box = &wireframeBox;

        if (tree.is_leaf) {
            box = &solidBox;
        }

        if (tree.is_leaf || level == selectedPath.size()) {
            box->setPosition(center.x(), center.y(), center.z());
            box->setScale(scale.x(), scale.y(), scale.z());
        }

        if (tree.is_leaf) {
            sb::Color col = valueToColor(tree.data.leaf.value);
            col.a = 0.1;
            box->setColor(col);
        }

        wnd.draw(*box);
        ++boxesDrawn;
    }

    void draw(const kd_tree<double> &tree)
    {
        wnd.clear(sb::Color(0.f, 0.f, 0.5f));

        wnd.draw(backgroundBox);

        boxesDrawn = 0;

        if (drawSelection) {
            drawTreeSelection(tree);
        } else {
            drawTree(tree);
        }

        size_t currLineIdx = 0;
        wnd.drawString(fpsString, { 0.0f, 0.0f },
                       (fpsCurrValue > 30.f
                           ? sb::Color::Green
                           : (fpsCurrValue > 20.f ? sb::Color::Yellow
                                                  : sb::Color::Red)),
                       currLineIdx++);
        wnd.drawString("selectedPath = " + selectedPath.toString(),
                       { 0.0f, 0.0f }, sb::Color::White, currLineIdx++);
        wnd.drawString("drawSelection = " + std::to_string(drawSelection),
                       { 0.0f, 0.0f }, sb::Color::White, currLineIdx++);
        wnd.drawString(std::to_string(boxesDrawn) + " boxes",
                       { 0.0f, 0.0f }, sb::Color::White, currLineIdx++);

        const kd_tree<double> *selected = selectedPath.apply(tree);
        if (selected) {
            wnd.drawString("value = " + (selected->is_leaf
                                         ? std::to_string(selected->data.leaf.value)
                                         : "<none>"),
                           { 0.0f, 0.0f }, sb::Color::White, currLineIdx++);
        }

        wnd.drawString("WSAD+mouse to move around, numpad 0/1/2 to select cells, space to toggle display",
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

        // An example kd-tree construction.

        // volume represented by the tree
        Bbox_3 kd_tree_box = {-1, -1, -1, 1, 1, 1};
        // approximated function
        auto function = [](double x, double y, double z) {
            return exp(-(x*x+y*y+z*z)/(2*0.1));
        };
        // maximum allowed error. Note: actual error depends on the accuracy
        // of the ErrorEstimator template parameter of the kd_tree.
        double required_accuracy = 0.2;

        typedef decltype(function(0,0,0)) ElementT;

        // tree construction
        tree = kd_tree<ElementT,
                       gradient_box_splitter<ElementT, 10>
                      >::build(kd_tree_box, function, required_accuracy);
    }

    TreeVisualizer visualizer;
    visualizer.show(*tree.get());

    for (double x = 0.0; x < 1.0; x += 0.1) {
        for (double y = 0.0; y < 1.0; y += 0.1) {
            for (double z = 0.0; z < 1.0; z += 0.1) {
                std::cout << x << " " << y << " " << z << " "
                          // kd-tree sampling example
                          << tree->value_at(x, y, z) << "\n";
            }
        }
        std::cout << "\n";
    }

    return 0;
}
