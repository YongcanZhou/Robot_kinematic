#ifndef ROBOT_H_
#define ROBOT_H_

#include <memory>
#include <aris.hpp>

namespace zyc
{
    namespace kinematic{
//        aris::dynamic::ModelBase* model;

        class Move : public aris::core::CloneObject<Move, aris::plan::Plan>{
            public:
                auto forward_kinematic()-> int;
//                auto prepareNrt()->void;
                auto inverse_kinematic()-> void;
                virtual ~Move();
                explicit Move(const std::string &name = "Move");
                ARIS_DEFINE_BIG_FOUR(Move)

            private:
                struct Imp;
//                aris::core::ImpPtr<Imp> imp_;
        };
        auto createPlanRoot()->std::unique_ptr<aris::plan::PlanRoot>;

    }
}
#endif
