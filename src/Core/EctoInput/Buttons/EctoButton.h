//
// Created by Abiel on 9/11/18.
//

#ifndef BOTBUSTERSREBIRTH_ECTOBUTTON_H
#define BOTBUSTERSREBIRTH_ECTOBUTTON_H

#include <mutex>
#include <frc2/command/button/Button.h>

class EctoButton : public frc2::Button {
public:
    EctoButton();
    EctoButton(EctoButton&& other);
    EctoButton(const EctoButton& other);

	/**
	 * Pass the raw button input to this
	 * @param status
	 */
	void updateStatus(bool status);
	
	/**
	 * Returns the processed input
	 * @return
	 */
	bool get() const;

    ~EctoButton();

protected:
	virtual bool calculateOutput(bool input);
	
	bool outValue = false;
	
	mutable std::mutex buttonMutex{};
};

#endif //BOTBUSTERSREBIRTH_ECTOBUTTON_H
