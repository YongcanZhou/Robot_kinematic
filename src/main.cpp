#include <aris.hpp>
#include <iostream>
#include <string>

#include "robot.h"

using namespace std;
using namespace aris::dynamic;
auto static xml_path = std::filesystem::absolute(".");	//获取当前工程所在的路径
auto static log_path = std::filesystem::absolute(".");	//获取当前工程所在的路径
const std::string xml_filename = "xb4.xml";		//控制配置文件名称
const std::string log_folder = "log";			//log文件夹名称

int main(int argc, char *argv[])
{
    PumaParam param;
//    param.a1=
    param.a1 = 0.04;
    param.a2 = 0.275;
    param.a3 = 0.025;
    param.d1 = 0.342;
    param.d3 = 0.0;
    param.d4 = 0.280;

    param.tool0_pe[2] = 0.073;

    auto m = aris::dynamic::createModelPuma(param);
    dynamic_cast<aris::dynamic::GeneralMotion &>(m->generalMotionPool().at(0)).setPoseType(GeneralMotion::PoseType::EULER313 );
    //foward kinematic
//    m->setOutputPos()
    double input[6]{0,0.,0.,0.,0.,0.}, output[6],link[6];
    m->setInputPos(input);
    m->forwardKinematics();

    m->getOutputPos(output);
    dsp(1,6,output);

    //inversr kinematic
    output[0]+=0.1;
    m->setOutputPos(output);
    if(m->inverseKinematics()){
        std::cout<<"inverse kinematic failed"<<std::endl;
    };//0成功 1失败

    m->getInputPos(input);
    for(int i=0;i<m->partPool().size();++i){
        m->partPool().at(i).getPe(link,"313");
        std::cout<<"link"<<i;
        dsp(1,6,link);
    }
    dsp(1,6,input);

//    std::cout<<aris::core::toXmlString(*m)<<std::endl;


/*
    //model.xml---------------------------------------------------------
    xml_path = xml_path / xml_filename;				//拼接控制器配置文件路径(注意后面空一行，不然很奇怪得不到xml_path)

    cout<<"xml_path: "<< xml_path<<endl;

    log_path = log_path / log_folder;				//拼接log文件夹路径

    cout<<"log_path: "<< log_path<<endl;

    auto&cs = aris::server::ControlServer::instance();
    aris::core::fromXmlFile(cs, xml_path);  //load xml file

    cs.resetPlanRoot(zyc::kinematic::createPlanRoot().release());//加载cmd配置
    cs.init();
    // 开启WebSocket/socket服务器
    cs.open();
    cs.start();
    //多态类型指针 转换类型
    zyc::kinematic::Move& move_command = dynamic_cast<zyc::kinematic::Move &>(cs.planRoot().planPool().at(
            cs.planRoot().planPool().size() - 1));
    move_command.forward_kinematic();
    move_command.inverse_kinematic();
    //等待终端输入函数，本函数不能去掉，否则实时线程和主线程都会结束
    cs.runCmdLine();*/

//old code---------------------------------------------------------
/*    auto&cs = aris::server::ControlServer::instance();// 定义cs

    auto master = new aris::control::EthercatMaster; // master

    aris::core::fromXmlFile(cs, "C:/Users/ZHOUYC/Desktop/contact/xb4.xml");//加载kaanh.xml配置
    std::cout << aris::core::toXmlString(cs.master()) << std::endl;

    cs.resetPlanRoot(robot::createPlanROSMotorTest().release());

    cs.init();
	//开启WebSocket/socket服务器//
    cs.open();
    cs.start();
	//等待终端输入函数，本函数不能去掉，否则实时线程和主线程都会结束//
    cs.runCmdLine();*/

//example code---------------------------------------------------------
/*    auto&cs = aris::server::ControlServer::instance();

    aris::control::EthercatController ec;
    ec.scan();
    std::cout<<aris::core::toXmlString(ec) << std::endl;

    aris::core::fromXmlFile(cs, xmlpath);//加载kaanh.xml配置

    cs.resetPlanRoot(robot::createPlanRoot().release());//加载cmd配置

    cs.init();									//初始化
//    auto &roberr = kaanh::RobotError::get_instance();
//    roberr.InitErrorConfig();
    aris::core::logDirectory(logpath);			//设置log路径
//    std::cout << aris::core::toXmlString(cs) << std::endl; //print the control server state

    std::array<double, 6>f = {400,400,400,16,16,16};
//    FS_LIMITED.store(f);

    auto &cal = cs.model().calculator();		//UI变量求解器
    kaanhconfig::createUserDataType(cal, 6);		//预定义UI界面变量集
//    kaanhconfig::createPauseTimeSpeed();		//初始化UI停止暂停功能参数
    cs.start();   // 不注释，则程序运行时开启控制器服务*/

	return 0;
}
