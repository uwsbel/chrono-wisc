// =============================================================================
// PROJECT CHRONO - http://projectchrono.org
//
// Copyright (c) 2023 projectchrono.org
// All rights reserved.
//
// Use of this source code is governed by a BSD-style license that can be found
// in the LICENSE file at the top level of the distribution and at
// http://projectchrono.org/license-chrono.txt.
//
// =============================================================================
// Authors: Radu Serban, Marcel Offermans
// =============================================================================
//
// Irrlicht-based GUI driver for the a vehicle. This class implements the
// functionality required by its base ChDriver class using keyboard or joystick
// inputs. If a joystick is present it will use that as an input; it will
// otherwise default to a keyboard input.
// As an Irrlicht event receiver, its OnEvent() callback is used to keep track
// and update the current driver inputs. As such it does not need to override
// the default no-op Advance() virtual method.
//
// =============================================================================

#include <algorithm>
#include <fstream>
#include <iostream>
#include <sstream>
#include <climits>
#include <bitset>

#include "chrono_vehicle/driver/ChInteractiveDriverIRR.h"
#include "chrono_vehicle/utils/ChUtilsJSON.h"
#include "chrono/utils/ChUtils.h"

#include "chrono_thirdparty/filesystem/path.h"

using namespace irr;

namespace chrono {
namespace vehicle {

// -----------------------------------------------------------------------------

// Process joystick events every 16 ticks (~60 Hz)
const int JoystickProcessFrequency = 16;

// Output joystick debug info every 30 ticks
const int JoystickOutputFrequency = 32;

// -----------------------------------------------------------------------------

ChInteractiveDriverIRR::ChInteractiveDriverIRR(ChVehicleVisualSystemIrrlicht& vsys)
    : ChInteractiveDriver(*vsys.m_vehicle),
      m_vsys(vsys),
      m_joystick_debug(false),
      m_joystick_debug_frame(0),
      m_joystick_proccess_frame(0),
      m_callback_button(-1),
      m_callback_function(nullptr),
      m_joystick_file(GetDataFile("joystick/controller_Default.json")) {
    // Add this driver as an Irrlicht event receiver
    vsys.AddUserEventReceiver(this);

    // Activate joysticks, is any
    m_joystick_info = core::array<SJoystickInfo>();
    vsys.GetDevice()->activateJoysticks(m_joystick_info);
    if (m_joystick_info.size() > 0) {
        m_mode = InputMode::JOYSTICK;
    }
}

// -----------------------------------------------------------------------------

bool ChInteractiveDriverIRR::OnEvent(const SEvent& event) {
    if (event.EventType == EET_KEY_INPUT_EVENT && !event.KeyInput.PressedDown) {
        switch (event.KeyInput.Key) {
            case KEY_KEY_L:
                if (m_mode != InputMode::LOCK) {
                    m_mode = InputMode::LOCK;
                }
                return true;

            case KEY_KEY_K:
                if (m_mode != InputMode::KEYBOARD) {
                    m_throttle = 0;
                    m_steering = 0;
                    m_braking = 0;
                    m_clutch = 0;
                    m_mode = InputMode::KEYBOARD;
                }
                return true;

            case KEY_KEY_J:
                if (m_mode != InputMode::JOYSTICK && m_joystick_info.size() > 0) {
                    m_mode = InputMode::JOYSTICK;
                }
                return true;

            case KEY_KEY_F:
                if (m_mode != InputMode::DATAFILE && m_data_driver) {
                    m_mode = InputMode::DATAFILE;
                    m_time_shift = m_vsys.m_vehicle->GetSystem()->GetChTime();
                }
                return true;

            default:
                break;
        }
    }

    if (m_mode == InputMode::JOYSTICK && event.EventType == EET_JOYSTICK_INPUT_EVENT) {
        return ProcessJoystickEvents(event);
    }

    if (m_mode == InputMode::KEYBOARD && event.EventType == EET_KEY_INPUT_EVENT) {
        return ProcessKeyboardEvents(event);
    }

    return false;
}

bool ChInteractiveDriverIRR::ProcessJoystickEvents(const SEvent& event) {
    // Driver only handles input every 16 ticks (~60 Hz)
    if (m_joystick_proccess_frame < JoystickProcessFrequency) {
        m_joystick_proccess_frame++;
        return true;
    }

    m_joystick_proccess_frame = 0;

    // Update steering, throttle and brake axes...
    SetSteering(steerAxis.GetValue(event.JoystickEvent));
    SetThrottle(throttleAxis.GetValue(event.JoystickEvent));
    SetBraking(brakeAxis.GetValue(event.JoystickEvent));
    SetClutch(clutchAxis.GetValue(event.JoystickEvent));

    // joystick callback
    if (m_callback_button > -1 && m_callback_function != nullptr &&
        event.JoystickEvent.IsButtonPressed(m_callback_button)) {
        m_callback_function();
    }

    auto transmission = m_vehicle.GetTransmission();
    if (!transmission)
        return true;

    auto transmission_auto = transmission->asAutomatic();  // nullptr for a manual transmission
    auto transmission_manual = transmission->asManual();   // nullptr for an automatic transmission

    // Automatic transmission: check shift to manumatic and gear shift
    if (transmission->IsAutomatic()) {
        // Toggle between a automatic and manumatic shift modes
        if (toggleManualGearboxButton.IsPressed(event.JoystickEvent)) {
            if (transmission_auto->GetShiftMode() == ChAutomaticTransmission::ShiftMode::AUTOMATIC) {
                transmission_auto->SetShiftMode(ChAutomaticTransmission::ShiftMode::MANUAL);
            } else {
                transmission_auto->SetShiftMode(ChAutomaticTransmission::ShiftMode::AUTOMATIC);
            }
        }

        // Shift up or down
        if (shiftUpButton.IsPressed(event.JoystickEvent)) {
            transmission_auto->ShiftUp();
        } else if (shiftDownButton.IsPressed(event.JoystickEvent)) {
            transmission_auto->ShiftDown();
        }
    }

    // Manual transmission
    if (transmission->IsManual()) {
        // Sequential gear shifts: up or down
        if (shiftUpButton.IsPressed(event.JoystickEvent)) {
            transmission_manual->ShiftUp();
        } else if (shiftDownButton.IsPressed(event.JoystickEvent)) {
            transmission_manual->ShiftDown();
        }
        // Support an H-shifter if present and change gears if you press
        // the clutch and shift the car into a specific gear.
        if (clutchAxis.axis != ChJoystickAxisIRR::NONE) {
            double clutchPosition = clutchAxis.GetValue(event.JoystickEvent);
            // Check if the clutch is pressed...
            if ((clutchAxis.scaled_max - clutchPosition) < 0.1) {
                bool reverseGearEngaged = gearReverseButton.IsPressed(event.JoystickEvent);
                int forwardGearEngaged = 0;
                if (gear1Button.IsPressed(event.JoystickEvent, true))
                    forwardGearEngaged = 1;
                else if (gear2Button.IsPressed(event.JoystickEvent, true))
                    forwardGearEngaged = 2;
                else if (gear3Button.IsPressed(event.JoystickEvent, true))
                    forwardGearEngaged = 3;
                else if (gear4Button.IsPressed(event.JoystickEvent, true))
                    forwardGearEngaged = 4;
                else if (gear5Button.IsPressed(event.JoystickEvent, true))
                    forwardGearEngaged = 5;
                else if (gear6Button.IsPressed(event.JoystickEvent, true))
                    forwardGearEngaged = 6;
                else if (gear7Button.IsPressed(event.JoystickEvent, true))
                    forwardGearEngaged = 7;
                else if (gear8Button.IsPressed(event.JoystickEvent, true))
                    forwardGearEngaged = 8;
                else if (gear9Button.IsPressed(event.JoystickEvent, true))
                    forwardGearEngaged = 9;

                if (reverseGearEngaged) {
                    /// Gear is set to reverse
                    m_vsys.m_vehicle->GetTransmission()->SetGear(-1);
                } else if (forwardGearEngaged > 0) {
                    // All 'forward' gears set drive mode to forward, regardless of gear
                    m_vsys.m_vehicle->GetTransmission()->SetGear(forwardGearEngaged);
                }
            }
        }
    }

    // Output some debug information (can be very useful when setting up or calibrating joysticks)
    if (m_joystick_debug) {
        if (m_joystick_debug_frame < JoystickOutputFrequency) {
            m_joystick_debug_frame++;
        } else {
            m_joystick_debug_frame = 0;
            std::string joystickLine = "Joystick " + std::to_string(event.JoystickEvent.Joystick) + " A:";
            for (int i = 0; i < 6; i++) {
                joystickLine += " " + std::to_string(event.JoystickEvent.Axis[i]);
            }
            joystickLine += " B: " + std::bitset<32>(event.JoystickEvent.ButtonStates).to_string() + "\n";
            std::cout << joystickLine;
        }
    }

    return true;
}

bool ChInteractiveDriverIRR::ProcessKeyboardEvents(const SEvent& event) {
    ChAutomaticTransmission* transmission_auto = nullptr;
    ChManualTransmission* transmission_manual = nullptr;
    if (m_vehicle.GetTransmission()) {
        transmission_auto = m_vehicle.GetTransmission()->asAutomatic();  // nullptr for a manual transmission
        transmission_manual = m_vehicle.GetTransmission()->asManual();   // nullptr for an automatic transmission
    }

    if (event.KeyInput.PressedDown) {
        switch (event.KeyInput.Key) {
            case KEY_KEY_A:
                m_steering_target = ChClamp(m_steering_target + m_steering_delta, -1.0, +1.0);
                return true;
            case KEY_KEY_D:
                m_steering_target = ChClamp(m_steering_target - m_steering_delta, -1.0, +1.0);
                return true;
            case KEY_KEY_W:
                m_throttle_target = ChClamp(m_throttle_target + m_throttle_delta, 0.0, +1.0);
                if (m_throttle_target > 0)
                    m_braking_target = ChClamp(m_braking_target - m_braking_delta * 3, 0.0, +1.0);
                return true;
            case KEY_KEY_S:
                m_throttle_target = ChClamp(m_throttle_target - m_throttle_delta * 3, 0.0, +1.0);
                if (m_throttle_target <= 0)
                    m_braking_target = ChClamp(m_braking_target + m_braking_delta, 0.0, +1.0);
                return true;
            default:
                break;
        }
        if (transmission_manual) {
            switch (event.KeyInput.Key) {
                case KEY_KEY_E:
                    m_clutch_target = ChClamp(m_clutch_target + m_clutch_delta, 0.0, +1.0);
                    return true;
                case KEY_KEY_Q:
                    m_clutch_target = ChClamp(m_clutch_target - m_clutch_delta, 0.0, +1.0);
                    return true;
                default:
                    break;
            }
        }
    } else {
        switch (event.KeyInput.Key) {
            case KEY_KEY_C:
                m_steering_target = 0;
                return true;
            case KEY_KEY_R:
                m_throttle_target = 0;
                m_braking_target = 0;
                m_clutch_target = 0;
                return true;
            default:
                break;
        }
        if (transmission_auto) {
            switch (event.KeyInput.Key) {
                case KEY_KEY_Z:
                    if (transmission_auto->GetDriveMode() != ChAutomaticTransmission::DriveMode::FORWARD)
                        transmission_auto->SetDriveMode(ChAutomaticTransmission::DriveMode::FORWARD);
                    else
                        transmission_auto->SetDriveMode(ChAutomaticTransmission::DriveMode::REVERSE);
                    return true;
                case KEY_KEY_X:
                    transmission_auto->SetDriveMode(ChAutomaticTransmission::DriveMode::NEUTRAL);
                    return true;
                case KEY_KEY_T:
                    if (transmission_auto->GetShiftMode() == ChAutomaticTransmission::ShiftMode::MANUAL)
                        transmission_auto->SetShiftMode(ChAutomaticTransmission::ShiftMode::AUTOMATIC);
                    else
                        transmission_auto->SetShiftMode(ChAutomaticTransmission::ShiftMode::MANUAL);
                    return true;
                case KEY_OEM_6:  // ']'
                    transmission_auto->ShiftUp();
                    return true;
                case KEY_OEM_4:  // '['
                    transmission_auto->ShiftDown();
                    return true;
                default:
                    break;
            }
        } else if (transmission_manual) {
            switch (event.KeyInput.Key) {
                case KEY_OEM_6:  // ']'
                    transmission_manual->ShiftUp();
                    return true;
                case KEY_OEM_4:  // '['
                    transmission_manual->ShiftDown();
                    return true;
                default:
                    break;
            }
        }
    }

    return false;
}

// -----------------------------------------------------------------------------

void ChInteractiveDriverIRR::SetButtonCallback(int button, void (*cbfun)()) {
    m_callback_function = cbfun;
    m_callback_button = button;
}

void ChInteractiveDriverIRR::SetJoystickConfigFile(const std::string& filename) {
    if (!filesystem::path(filename).exists()) {
        std::cerr << "Error: the specified joystick configuration file " << filename << " does not exist.\n"
                  << "Using default configuration." << std::endl;
        return;
    }
    m_joystick_file = filename;
}

// -----------------------------------------------------------------------------

void ChInteractiveDriverIRR::Initialize() {
    if (m_joystick_info.size() > 0) {
        rapidjson::Document d;
        ReadFileJSON(m_joystick_file, d);

        if (d.IsNull()) {
            std::cerr << "Error reading joystick configuration file " << m_joystick_file << "\n"
                      << "Reverting to KEYBOARD mode" << std::endl;
            m_mode = InputMode::KEYBOARD;
            return;
        }

        // Read specification for individual axes and buttons.
        // For any axis/button listed in the specification file, default to first joystick
        steerAxis.Read(d, "steering", m_joystick_debug);
        throttleAxis.Read(d, "throttle", m_joystick_debug);
        brakeAxis.Read(d, "brake", m_joystick_debug);
        clutchAxis.Read(d, "clutch", m_joystick_debug);
        shiftUpButton.Read(d, "shiftUp", m_joystick_debug);
        shiftDownButton.Read(d, "shiftDown", m_joystick_debug);
        gearReverseButton.Read(d, "gearReverse", m_joystick_debug);
        gear1Button.Read(d, "gear1", m_joystick_debug);
        gear2Button.Read(d, "gear2", m_joystick_debug);
        gear3Button.Read(d, "gear3", m_joystick_debug);
        gear4Button.Read(d, "gear4", m_joystick_debug);
        gear5Button.Read(d, "gear5", m_joystick_debug);
        gear6Button.Read(d, "gear6", m_joystick_debug);
        gear7Button.Read(d, "gear7", m_joystick_debug);
        gear8Button.Read(d, "gear8", m_joystick_debug);
        gear9Button.Read(d, "gear9", m_joystick_debug);
        toggleManualGearboxButton.Read(d, "toggleManualGearbox", m_joystick_debug);

        // Loop over available controllers and distribute axes per controller if specified
        for (u32 id = 0; id < m_joystick_info.size(); ++id) {
            const auto& jinfo = m_joystick_info[id];
            std::cout << "Joystick " << id << ":\n";
            std::cout << "\tName: '" << jinfo.Name.c_str() << "'\n";
            std::cout << "\tAxes: " << jinfo.Axes << "\n";
            std::cout << "\tButtons: " << jinfo.Buttons << "\n";
            std::cout << "\tHat is: ";
            switch (jinfo.PovHat) {
                case SJoystickInfo::POV_HAT_PRESENT:
                    std::cout << "present\n";
                    break;
                case SJoystickInfo::POV_HAT_ABSENT:
                    std::cout << "absent\n";
                    break;
                case SJoystickInfo::POV_HAT_UNKNOWN:
                default:
                    std::cout << "unknown\n";
                    break;
            }

            // Now try to assign any axis and buttons to this joystick...
            if (strcmp(throttleAxis.name.c_str(), jinfo.Name.c_str()) == 0) {
                throttleAxis.id = id;
            }
            if (strcmp(brakeAxis.name.c_str(), jinfo.Name.c_str()) == 0) {
                brakeAxis.id = id;
            }
            if (strcmp(clutchAxis.name.c_str(), jinfo.Name.c_str()) == 0) {
                clutchAxis.id = id;
            }
            if (strcmp(steerAxis.name.c_str(), jinfo.Name.c_str()) == 0) {
                steerAxis.id = id;
            }
            if (strcmp(shiftUpButton.name.c_str(), jinfo.Name.c_str()) == 0) {
                shiftUpButton.id = id;
            }
            if (strcmp(shiftDownButton.name.c_str(), jinfo.Name.c_str()) == 0) {
                shiftDownButton.id = id;
            }
            if (strcmp(gear1Button.name.c_str(), jinfo.Name.c_str()) == 0) {
                gear1Button.id = id;
            }
            if (strcmp(gear2Button.name.c_str(), jinfo.Name.c_str()) == 0) {
                gear2Button.id = id;
            }
            if (strcmp(gear3Button.name.c_str(), jinfo.Name.c_str()) == 0) {
                gear3Button.id = id;
            }
            if (strcmp(gear4Button.name.c_str(), jinfo.Name.c_str()) == 0) {
                gear4Button.id = id;
            }
            if (strcmp(gear5Button.name.c_str(), jinfo.Name.c_str()) == 0) {
                gear5Button.id = id;
            }
            if (strcmp(gear6Button.name.c_str(), jinfo.Name.c_str()) == 0) {
                gear6Button.id = id;
            }
            if (strcmp(gear7Button.name.c_str(), jinfo.Name.c_str()) == 0) {
                gear7Button.id = id;
            }
            if (strcmp(gear8Button.name.c_str(), jinfo.Name.c_str()) == 0) {
                gear8Button.id = id;
            }
            if (strcmp(gear9Button.name.c_str(), jinfo.Name.c_str()) == 0) {
                gear9Button.id = id;
            }
            if (strcmp(gearReverseButton.name.c_str(), jinfo.Name.c_str()) == 0) {
                gearReverseButton.id = id;
            }
            if (strcmp(toggleManualGearboxButton.name.c_str(), jinfo.Name.c_str()) == 0) {
                toggleManualGearboxButton.id = id;
            }
        }
    }
}

// -----------------------------------------------------------------------------

ChJoystickAxisIRR::ChJoystickAxisIRR()
    : id(-1), name("Unknown"), axis(Axis::NONE), min(0), max(1), scaled_min(0), scaled_max(1), value(0) {}

double ChJoystickAxisIRR::GetValue(const irr::SEvent::SJoystickEvent& joystickEvent) {
    if (joystickEvent.Joystick == id) {
        // Scale raw_value fromm [scaled_min, scaled_max] to [min, max] range
        value = (joystickEvent.Axis[axis] - max) * (scaled_max - scaled_min) / (max - min) + scaled_max;
    }
    return value;
}

void ChJoystickAxisIRR::Read(rapidjson::Document& d, const std::string& elementName, bool dbg_print) {
    auto element = elementName.c_str();
    if (d.HasMember(element) && d[element].IsObject()) {
        name = d[element]["name"].GetString();
        id = 0;  // default to first attached controller
        axis = (Axis)d[element]["axis"].GetInt();
        min = d[element]["min"].GetDouble();
        max = d[element]["max"].GetDouble();
        scaled_min = d[element]["scaled_min"].GetDouble();
        scaled_max = d[element]["scaled_max"].GetDouble();
        value = min;
    } else if (dbg_print) {
        std::cerr << "Expected a joystick axis definition for " << elementName << " but did not find one." << std::endl;
    }
}

ChJoystickButtonIRR::ChJoystickButtonIRR()
    : id(-1), name("Unknown"), button(-1), buttonPressed(false), buttonPressedCount(0) {}

bool ChJoystickButtonIRR::IsPressed(const irr::SEvent::SJoystickEvent& joystickEvent, bool continuous) {
    if (joystickEvent.Joystick == id) {
        buttonPressed = joystickEvent.IsButtonPressed(button);
    }
    if (buttonPressed) {
        buttonPressedCount++;
    } else {
        buttonPressedCount = 0;
    }
    return continuous ? buttonPressedCount > 0 : buttonPressedCount == 1;
}

void ChJoystickButtonIRR::Read(rapidjson::Document& d, const std::string& elementName, bool dbg_print) {
    auto element = elementName.c_str();
    if (d.HasMember(element) && d[element].IsObject()) {
        id = 0;  // default to first attached controller
        name = d[element]["name"].GetString();
        button = d[element]["button"].GetInt();
    } else if (dbg_print) {
        std::cerr << "Expected a joystick button definition for " << elementName << " but did not find one."
                  << std::endl;
    }
}

}  // end namespace vehicle
}  // end namespace chrono
