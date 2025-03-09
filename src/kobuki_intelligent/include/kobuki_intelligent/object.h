#include <string.h>

struct Point
{
    Point(const short x_ = 0, const short y_ = 0) : x(x_), y(y_) {}
    short x;
    short y;
};

class Object
{
public:
    Point error;
    Point frame;
    std::string name;
    int x_real;
    int y_real;
    int z_real;
    float distance;
    bool exist;
};