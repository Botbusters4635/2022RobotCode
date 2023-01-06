//
// Created by cc on 18/02/22.
//

#include "LoadIntake.h"

LoadIntake::LoadIntake(const std::shared_ptr<Intake> &intake) {
    this->intake = intake;
    AddRequirements({intake.get()});

}

void LoadIntake::Initialize() {
    initialState = intake->getBeamBreak();
}
void LoadIntake::Execute() {
    ;
}
void LoadIntake::End(bool interrupted) {
    ;
}
bool LoadIntake::IsFinished() {
    if (initialState != intake->getBeamBreak()){
        return true;
    }else{return false;}
}