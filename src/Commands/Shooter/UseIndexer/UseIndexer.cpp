//
// Created by cc on 25/01/22.
//

#include "UseIndexer.h"




UseIndexer::UseIndexer(const std::shared_ptr<Indexer> &indexer, double targetVel, bool isInverted) {
    this->indexer = indexer;
    this->targetVel = targetVel;
    this->isInverted = isInverted;
}

void UseIndexer::Initialize() {
    ;
}

void UseIndexer::Execute() {
    ;
}

void UseIndexer::End(bool interrupted) {
    ;
}

bool UseIndexer::IsFinished() {
    return false;
}