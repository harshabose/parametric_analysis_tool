#include <iostream>

#include "include/parametricAnalysis.h"

int main()
{
    const auto parametric_operator = parametricAnalysis_h::parametric_analysis();
    std::cout << "Optimal T/W ratio is -> " << parametric_operator->parametricPoint[0] << std::endl;
    std::cout << "Optimal W/S ratio is -> " << parametric_operator->parametricPoint[1] << std::endl;
    return 0;
}
