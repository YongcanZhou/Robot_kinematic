#include <algorithm>
#include <array>
#include <cstdlib>
#include <string>
#include <bitset>
#include <cmath>
#include "robot.h"
#include <cinttypes>

using namespace aris::dynamic;
using namespace aris::plan;
const double PI = aris::PI;

namespace zyc::kinematic {

    struct MoveParam {double a;};
    struct Move::Imp : public MoveParam {};
    char eu_type[4]{'3', '2', '1', '\0'};

//    auto Move::prepareNrt() -> void {
//        std::cout<<"test";
//        return ;
//    }

    //forward kinematic calculation
    auto Move::forward_kinematic() -> int {
        auto &mout = master()->mout();
        // end-effector
        auto &ee = model()->generalMotionPool()[0];
        // motor_num
        int motor_num = model()->motionPool().size();

        for(std::size_t i =0; i < motor_num; i++) {
            model()->motionPool()[i].setMp(0.0);
        }

        if (model()->solverPool().at(1).kinPos()/*0逆解  1正解*/){// 成功回0
            std::cout<<"forward kinematic failed";
            return -1;
        }else{
            //screw_tool_center_point
            double s_tcp[6]{0,0,0,0,0,0};
            ee.getP(s_tcp);
            mout<<"screw_tool_center_point_position: "<<s_tcp[0]<<" "<<s_tcp[1]<<" "<<s_tcp[2]<<" "<<s_tcp[3]<<" "<<s_tcp[4]<<" "<<s_tcp[5]<<std::endl;
        }
    }

    //inverse kinematic calculation
    auto Move::inverse_kinematic() -> void {
        auto &mout = master()->mout();
        // end-effector
        auto &ee = model()->generalMotionPool()[0];
        // motor_num
        int motor_num = model()->motionPool().size();
        //screw_tool_center_point
        double s_tcp[6]{0.393,0,0.642,3.92699,1.5708,3.92699};

        ee.setP(s_tcp);
        model()->solverPool()[0].kinPos();//0逆解  1正解
        double joint[6];
        static double link[6][6];
        for(std::size_t i = 0; i<motor_num; i++)
        {
            model()->motionPool()[i].updP();
            joint[i] = model()->motionPool()[i].mp();
            model()->motionPool()[i].getP(link[i]);//get link position
            mout<<"link"<<i+1<<": "<<link[i][0]<<" "<<link[i][1]<<" "<<link[i][2]<<" "<<link[i][3]<<" "<<link[i][4]<<" "<<link[i][5]<<" "<<std::endl;
             /*model()->motorPool()[i].setTargetPos();// motor position
             model()->partPool().at(2).pm();//link position
             controller()->motionPool()[i].setTargetPos(x_joint[i]);*/
        }
        mout<<"joint_angle: "<<joint[0]<<" "<<joint[1]<<" "<<joint[2]<<" "<<joint[3]<<" "<<joint[4]<<" "<<joint[5]<<std::endl;
    }

    Move::~Move()=default;
    Move::Move(const std::string &name){
    /*        //构造函数参数说明，构造函数通过xml的格式定义本条指令的接口，name表示参数名，default表示输入参数，abbreviation表示参数名的缩写(缩写只能单个字符)
        //1 GroupParam下面的各个节点都是输入参数，如果没有给定会使用默认值
        //2 UniqueParam下面的各个节点互斥，有且只能使用其中的一个
        //3 例如，通过terminal或者socket发送“mvs --pos=0.1”，控制器实际会按照mvs --pos=0.1rad --time=1s --timenum=2 --all执行
        //command().loadXmlStr(
        aris::core::fromXmlString(command(),
                                  "<Command name=\"mvs\">"
                                  "	<GroupParam>"
                                  "		<Param name=\"pos\" default=\"current_pos\"/>"
                                  "		<Param name=\"time\" default=\"1.0\" abbreviation=\"t\"/>"
                                  "		<Param name=\"timenum\" default=\"2\" abbreviation=\"n\"/>"
                                  "		<UniqueParam default=\"all\">"\
            "			<Param name=\"all\" abbreviation=\"a\"/>"\
            "			<Param name=\"motion_id\" abbreviation=\"m\" default=\"0\"/>"
                                  "		</UniqueParam>"
                                  "	</GroupParam>"
                                  "</Command>");*/
    }

    auto createPlanRoot()->std::unique_ptr<aris::plan::PlanRoot>
    {
        std::unique_ptr<aris::plan::PlanRoot> plan_root(new aris::plan::PlanRoot);

        plan_root->planPool().add<aris::plan::Enable>();
        plan_root->planPool().add<aris::plan::Disable>();
        plan_root->planPool().add<aris::plan::Home>();
        plan_root->planPool().add<aris::plan::Mode>();
        plan_root->planPool().add<aris::plan::Show>();
        plan_root->planPool().add<aris::plan::Sleep>();
        plan_root->planPool().add<aris::plan::Clear>();
        plan_root->planPool().add<aris::plan::Recover>();
        auto &rs = plan_root->planPool().add<aris::plan::Reset>();
        rs.command().findParam("pos")->setDefaultValue("{0.5,0.392523364485981,0.789915966386555,0.5,0.5,0.5}");

        auto &mvaj = plan_root->planPool().add<aris::plan::MoveAbsJ>();
        mvaj.command().findParam("vel")->setDefaultValue("0.1");

        plan_root->planPool().add<aris::plan::MoveL>();
        plan_root->planPool().add<aris::plan::MoveJ>();
        plan_root->planPool().add<aris::plan::GetXml>();
        plan_root->planPool().add<aris::plan::SetXml>();
        plan_root->planPool().add<aris::plan::Start>();
        plan_root->planPool().add<aris::plan::Stop>();

        //添加自己写的命令
        plan_root->planPool().add<Move>();

        return plan_root;
    }
}