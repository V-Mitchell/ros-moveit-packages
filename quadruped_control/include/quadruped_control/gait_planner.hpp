/*  Author: Victor M
*   Email: victorcmitchell@gmail.com
*/

#include <utility>
#include <vector>

#ifndef GAIT_PLANNER
#define GAIT_PLANNER

class gait_planner {
    public:
        gait_planner(double delta = 0.0005) {
            t_last = 0;
            s = 0;
            this->delta = delta;
        }

    private:
        double t_last;
        double s;
        double delta; // virtual displacement in stance phase
        std::vector<std::pair<double, double> > nodes; // bezier nodes


};

#endif