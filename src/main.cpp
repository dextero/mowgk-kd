#include <fenv.h>
#include <iostream>

#include <sandbox/window/window.h>

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
        mRunning(false)
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


class TreeVisualizer
{
public:
    TreeVisualizer():
        wnd(1440, 900)
    {}

    void show(const kd_tree<double> &)
    {
        while (wnd.isOpened()) {
            handleInput();
            draw();
        }
    }

private:
    sb::Window wnd;

    void handleInput()
    {
        sb::Event e;
        while (wnd.getEvent(e))
        {
            switch (e.type)
            {
            case sb::Event::KeyPressed:
                switch (e.data.key) {
                case sb::Key::Esc:
                    wnd.close();
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

    void draw()
    {
        wnd.clear(sb::Color(0.f, 0.f, 0.5f));
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
