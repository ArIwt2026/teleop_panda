#include <franka/robot.h>
#include <franka/exception.h>
#include <exception>

extern "C" {
    bool check_fci(const char* ip) {
        try {
            franka::Robot robot(ip, franka::RealtimeConfig::kIgnore);
            return true;
        } catch (const franka::Exception& e) {
            return false;
        } catch (const std::exception& e) {
            return false;
        } catch (...) {
            return false;
        }
    }
}
