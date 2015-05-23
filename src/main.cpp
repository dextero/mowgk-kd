#include <iostream>

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


int main(int /*argc*/,
         char* /*argv*/[])
{
    std::unique_ptr<kd_tree<double>> tree;

    {
        ScopedTimer timer("generating k-d tree");
        tree = kd_tree<double>::build({0,0,0,1,1,1},
                                      [](double x, double y, double z) {
                                          return x + y + z;
                                      },
                                      0.25);
    }

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
