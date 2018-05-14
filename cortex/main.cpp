
#include "asio.hpp"
#include "types.hpp"
#include "util.hpp"

#include "controller.hpp"
#include "driver.hpp"
#include "steering.hpp"

#include <fmt/format.h>
//MQTT include
#include <iomanip>
#include <map>

#include <boost/lexical_cast.hpp>

#include <mqtt_client_cpp.hpp>

#define PORT 1111
#define IPADDRESS 127.0.0.1

int main()
{
	//MQTT
	boost::asio::io_service ios;
	
	std::uint16_t pid_sub1;
    	std::uint16_t pid_sub2;
	int count;
	
	auto c = mqtt::make_client(ios, PORT, boost::lexical_cast<std::uint16_t>(IPADDRESS));
	
	auto disconnect = [&] {
        if (++count == 5) c->disconnect();
        };
	
	 // Setup client
        c->set_client_id("cid1");
        c->set_clean_session(true);
	
	// Setup handlers
    c->set_connack_handler(
        [&c, &pid_sub1, &pid_sub2]
        (bool sp, std::uint8_t connack_return_code){
            std::cout << "Connack handler called" << std::endl;
            std::cout << "Clean Session: " << std::boolalpha << sp << std::endl;
            std::cout << "Connack Return Code: "
                      << mqtt::connect_return_code_to_str(connack_return_code) << std::endl;
            if (connack_return_code == mqtt::connect_return_code::accepted) {
                pid_sub1 = c->subscribe("mqtt_client_cpp/topic1", mqtt::qos::at_most_once);
                pid_sub2 = c->subscribe("mqtt_client_cpp/topic2_1", mqtt::qos::at_least_once,
                                       "mqtt_client_cpp/topic2_2", mqtt::qos::exactly_once);
            }
            return true;
        });
    c->set_close_handler(
        []
        (){
            std::cout << "closed." << std::endl;
        });
    c->set_error_handler(
        []
        (boost::system::error_code const& ec){
            std::cout << "error: " << ec.message() << std::endl;
        });
    c->set_puback_handler(
        [&]
        (std::uint16_t packet_id){
            std::cout << "puback received. packet_id: " << packet_id << std::endl;
            disconnect();
            return true;
        });
    c->set_pubrec_handler(
        []
        (std::uint16_t packet_id){
            std::cout << "pubrec received. packet_id: " << packet_id << std::endl;
            return true;
        });
    c->set_pubcomp_handler(
        [&]
        (std::uint16_t packet_id){
            std::cout << "pubcomp received. packet_id: " << packet_id << std::endl;
            disconnect();
            return true;
        });
    c->set_suback_handler(
        [&]
        (std::uint16_t packet_id, std::vector<boost::optional<std::uint8_t>> results){
            std::cout << "suback received. packet_id: " << packet_id << std::endl;
            for (auto const& e : results) {
                if (e) {
                    std::cout << "subscribe success: " << mqtt::qos::to_str(*e) << std::endl;
                }
                else {
                    std::cout << "subscribe failed" << std::endl;
                }
            }
            if (packet_id == pid_sub1) {
                c->publish_at_most_once("mqtt_client_cpp/topic1", "test1");
            }
            else if (packet_id == pid_sub2) {
                c->publish_at_least_once("mqtt_client_cpp/topic2_1", "test2_1");
                c->publish_exactly_once("mqtt_client_cpp/topic2_2", "test2_2");
            }
            return true;
        });
    c->set_publish_handler(
        [&]
        (std::uint8_t header,
         boost::optional<std::uint16_t> packet_id,
         std::string topic_name,
         std::string contents){
            std::cout << "publish received. "
                      << "dup: " << std::boolalpha << mqtt::publish::is_dup(header)
                      << " pos: " << mqtt::qos::to_str(mqtt::publish::get_qos(header))
                      << " retain: " << mqtt::publish::is_retain(header) << std::endl;
            if (packet_id)
                std::cout << "packet_id: " << *packet_id << std::endl;
            std::cout << "topic_name: " << topic_name << std::endl;
            std::cout << "contents: " << contents << std::endl;
            disconnect();
            return true;
        });

    // Connect
    //c->connect();

    //ios.run();
	//MQTT
	io_context ioctx;

	Controller ctrl(ioctx, "/dev/input/js0");
	Driver driver(ioctx, "/dev/ttyACM0");
	Steering steering(ioctx);

	ctrl.on_axis = [&](u32, Controller::Axis num, i16 val)
	{
		static std::unordered_map<Controller::Axis, i16> input_state
		{
			{ Controller::LS_H, 0 },
			{ Controller::LT2, Controller::min },
			{ Controller::RT2, Controller::min },
		};

		constexpr i32 axis_min = Controller::min, axis_max = Controller::max;

		auto i = input_state.find(num);
		if(i == input_state.end())
			return;

		i->second = val;

		// motor control
		//###############
		i32 input = input_state[Controller::RT2] - input_state[Controller::LT2];
		static i32 motor_input_prev = 0;
		if(motor_input_prev != input)
		{
			motor_input_prev = input;

			u8 speed = Driver::Speed::STOP;
			if(input < 0)
				speed = map<u8>(input, 0, axis_min*2, Driver::Speed::STOP, Driver::Speed::BACK_FULL);
			else
				speed = map<u8>(input, 0, axis_max*2, Driver::Speed::STOP, Driver::Speed::FORWARD_FULL);

			static u8 speed_prev = Driver::Speed::STOP;
			if(speed != speed_prev)
			{
				speed_prev = speed;
				fmt::print("MOTOR: {:5} => {:02x}\n", input, speed);
				driver.drive(speed);
			}
		}

		// steer control
		//###############
		static i16 steer_input_prev = 0;
		i16 &steer = input_state[Controller::LS_H];
		if(steer_input_prev != steer)
		{
			steer_input_prev = steer;
			steering.steer(steer);
		}
	};

	ioctx.run();

	return 0;
}
