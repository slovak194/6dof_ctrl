#include <iostream>

#include <se3.hpp>
#include <sim3.hpp>

int main() {
    std::cout << "Hello, World!" << std::endl;

    auto se3 = Sophus::SE3d();
    std::cout << se3.matrix() << std::endl;

    se3 = se3.trans(1, 2, 8);

    se3 = se3.rotX(M_PI_4);

    std::cout << se3.matrix() << std::endl;

    std::cout << se3.so3().unit_quaternion().coeffs().eval() << std::endl;

    std::cout << se3.Adj() << std::endl;

    return 0;
}


