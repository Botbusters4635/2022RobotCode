//
// Created by abiel on 2/12/22.
//

#ifndef BOTBUSTERS_REBIRTH_SSHCONNECTIONTEST_H
#define BOTBUSTERS_REBIRTH_SSHCONNECTIONTEST_H

#include <wpi/TCPConnector.h>
#include <wpi/Logger.h>
#include "Core/EctoChecklist/ChecklistItem.h"

class SSHConnectionTest : public ChecklistItem {
public:
    SSHConnectionTest(const std::string &hostname);

    void Execute() override;

    bool IsFinished() override;

private:
    std::string hostname;
    bool finished{false};
};


#endif //BOTBUSTERS_REBIRTH_SSHCONNECTIONTEST_H
