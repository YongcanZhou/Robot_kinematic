#include <aris.hpp>
#include <iostream>

using namespace std;
using namespace aris::dynamic;

int main(int argc, char *argv[]) {
    std::cout << "inverse kinematic failed" << std::endl;
    PumaParam param;
    param.a1 = 0.04;
    param.a2 = 0.275;
    param.a3 = 0.025;
    param.d1 = 0.342;
    param.d3 = 0.0;
    param.d4 = 0.280;

    param.tool0_pe[2] = 0.073;

    auto m = aris::dynamic::createModelPuma(param);
    dynamic_cast<aris::dynamic::GeneralMotion &>(m->generalMotionPool().at(0)).setPoseType(
            GeneralMotion::PoseType::EULER313);
    //forward kinematic
    double input[6]{0, 0., 0., 0., 0., 0.}, output[6], link[6];
    m->setInputPos(input);
    m->forwardKinematics();

    m->getOutputPos(output);
    dsp(1, 6, output);

    //inverse kinematic
    output[0] += 0.1;
    m->setOutputPos(output);
    if (m->inverseKinematics()) {
        std::cout << "inverse kinematic failed" << std::endl;
    }//0成功 1失败

    m->getInputPos(input);
    for (int i = 0; i < m->partPool().size(); ++i) {
        m->partPool().at(i).getPe(link, "313");
        std::cout << "link" << i;
        dsp(1, 6, link);
    }
    dsp(1, 6, input);

    return 0;

}