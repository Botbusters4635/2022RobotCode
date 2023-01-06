#ifndef BOTBUSTERSREBIRTH_ECTOINPUT_H
#define BOTBUSTERSREBIRTH_ECTOINPUT_H

#include <Core/EctoInput/Buttons/EctoButton.h>
#include <Core/EctoInput/Axis/JoystickAxis.h>
#include <Core/EctoModule/Manager.h>
#include <frc/DriverStation.h>
#include <frc/Joystick.h>
#include <map>
#include <utility>
#include <string>
#include <mutex>
#include <thread>
#include <chrono>

/**
 * This class is in charge of managing and updating all EctoButton and JoystickAxis classes for use within other systems.
 */
class InputManager : public Manager {

	struct JoystickData {
		std::shared_ptr<frc::Joystick> joystick;
		std::map<std::string, std::vector<EctoButton *>> registeredButtons;
		std::map<std::string, std::vector<JoystickAxis *>> registeredAxes;
        std::map<std::string, std::vector<EctoButton *>> registeredDPadButtons;
	};

public:
    static InputManager &getInstance() {
        static InputManager instance;
        return instance;
    }
	struct JoystickMap {
		std::map<std::string, int> axesMapping;
		std::map<std::string, int> buttonsMapping;
        std::map<std::string, int> dPadMapping;
	};
	
	void registerButton(EctoButton &button, const std::string &buttonName, int joystickId = 0);

    void unregisterButton(EctoButton *button);
	
	void registerAxis(JoystickAxis &axis, const std::string &axisName, int joystickId = 0);
	
	/**
	 * Rumble values should be from 0 to 1
	 */
	void setControllerRumble(double leftRumble, double rightRumble, int joystickId = 0);
	
	int getPOVAngleReading(int joystickId = 0);

    void registerDPadButton(EctoButton &button, const std::string &dPadButtonName = "up", int joystickId = 0);
	
	void addMapping(JoystickMap mapping, int joystickId = 0);

    void init() override;

    void update() override;

private:
	InputManager();
	
	InputManager &operator=(const InputManager &);
	
	std::map<int, JoystickMap> joystickMappings;
	std::map<int, JoystickData> joysticksData;

};

#endif
