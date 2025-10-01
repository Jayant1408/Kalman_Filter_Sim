#include "kf.hpp"
#include <iostream>

int main()
{
    KFConfig cfg;
    KalmanFilter1D kf(cfg);
    KFOutputs out = kf.run();

    //Save next to data/ at repo root (adjust relative path if needed)
    if(KalmanFilter1D::writeCSV("../../data/kf_1d_cpp.csv", out))
    {
        std::cout << "Saved CSV to data/kf_1d_cpp.csv\n";
    }
    else
    {
        std::cerr << "Failed to write CSV\n";
    }

    return 0;
}