/**************************************************************************
* Copyright 2023 Applied Robotics, Ltd.
***************************************************************************/

//подключение библииотек
#include <cstdio> 
#include <chrono>
#include <fstream>
#include <cstdlib> 
#include <unistd.h> 
#include <iostream>

#include "dynamixel_sdk.h"	// Подключение библиотеки DYNAMIXEL SDK
#include "fmt/format.h" 
#include <signal.h>

#define num_servos	5	// количество сервоприводов в манипуляторе
// Адреса сервоприводов на шине Dynamixel и модель:
#define DXL_ID_1	1	// MX-28T
#define DXL_ID_2	2	// MX-64T
#define DXL_ID_3	3	// MX-64T
#define DXL_ID_4	4	// MX-28T
#define DXL_ID_5	5	// AX-12A

#define DEVICENAME		"/dev/ttyS2" //шина для NanoPi
#define SERIAL_PUMP	"/dev/ttyS1" // OpenCM для NanoPi 
#define PROTOCOL	1.0		// версия протокола Dynamixel 
#define DXL_BAUDRATE		1000000	// скорость порта DXL 
#define SERIAL_BAUDRATE			115200		// скорость порта SERIAL

#define MIN_POSITION_LIMIT	0	// мин позиция в режиме По положению
#define MAX_POSITION_LIMIT	4095	// макс позиция в режиме По положению
#define TORQUE_ENABLE	1		// значения включения момента 
#define TORQUE_DISABLE	0	// значения выключения момента
#define START_POS5		1022 //середина диапазона пятого сервопривода
#define START_POS		2048 // Середина диапазона сервопривода 
#define SPEED	15		// Скорость передвижения сервопривода

// Адреса регистров Таблицы управления по протоколу 1.0:
#define ADDR_TORQUE_ENABLE	24	// чтение/запись размером 1 байт
#define ADDR_GOAL_POSITION    30    // ч/з 2 байт 
#define ADDR_PRESENT_POSITION 36   // ч 2 байт 
#define ADDR_PROFILE_VELOCITY 32 // ч/з 2 байт

#define num_states	10	// число состояний, в которые придет манипулятор
int states[num_states][num_servos] = {
{START_POS,	START_POS,	START_POS,	START_POS - 1000, START_POS5, },
{START_POS + 500, START_POS - 50, START_POS - 250, START_POS - 600, START_POS5, },
{START_POS + 500, START_POS - 150, START_POS - 380, START_POS - 300, START_POS5, },
{START_POS + 500, START_POS - 260, START_POS - 440, START_POS - 350, START_POS5, },// i=3 насос включен
{START_POS + 500, START_POS - 50, START_POS - 250, START_POS - 600, START_POS5, },
{START_POS - 600, START_POS - 50, START_POS - 250, START_POS - 600, START_POS5, },
{START_POS - 600, START_POS - 150, START_POS - 450, START_POS - 300, START_POS5, },
{START_POS - 600, START_POS - 200, START_POS - 450, START_POS - 350, START_POS5, },
{START_POS - 600, START_POS - 260, START_POS - 440, START_POS - 350, START_POS5, },// i=8 насос выключен
{START_POS - 600, START_POS - 50, START_POS - 250, START_POS - 600, START_POS5, },
};

uint8_t dxl_error = 0;	// код ошибки DYNAMIXEL
int dxl_comm_result = COMM_TX_FAIL;	// результат обмена данными

dynamixel::PortHandler* portHandlerSerial = dynamixel::PortHandler::getPortHandler(SERIAL_PUMP);// Определение объекта PortHandler, задание пути порта

//функция записи в Open CM состояния пневмосистемы
void pumpFunction(char cmd) {
	uint8_t c = cmd;
	portHandlerSerial->writePort(&c, 1);
}


// Функция включения/отключения (переменная cmd) момента 
void setTorque(uint8_t n_servo,uint8_t cmd, dynamixel::PacketHandler* packetH, dynamixel::PortHandler* portH) {
	for (int _id = 1; _id <= n_servo; _id++)
	{
		// Запись cmd в регистр ADDR_TORQUE_ENABLE 
		dxl_comm_result = packetH->write1ByteTxRx(portH, _id, ADDR_TORQUE_ENABLE, cmd, &dxl_error);
	}
}

volatile sig_atomic_t stop;	// сигнал для останова программы по ctrl+c

// Изменение переменной stop при получении сигнала 
void signalHandler(int signum) {
	stop = 1;
}

int main() {
	// Определение объекта PortHandler, задание пути порта
	// Получение методов и составляющих PortHandlerLinux или PortHandlerWindows
	dynamixel::PortHandler* portHandler = dynamixel::PortHandler::getPortHandler(DEVICENAME);

	// Определение объекта PacketHandler, задание версии протокола
	// Получение методов и составляющих Protocol1PacketHandler или Protocol2PacketHandler
	dynamixel::PacketHandler* packetHandler = dynamixel::PacketHandler::getPacketHandler(PROTOCOL);

	// Открытие порта
	if (portHandler->openPort()) {
		fmt::print("SUCCESS: open port{}\n", DEVICENAME);
	}
	else {
		fmt::print("FAILED: open port{}\n", DEVICENAME); return 0;
	}

	// Установка скорости порта DXL
	if (portHandler->setBaudRate(DXL_BAUDRATE)) {
		fmt::print("SUCCESS: set baudrate to{}\n", DXL_BAUDRATE);
	}
	else {
		fmt::print("FAILED: set baudrate to{}\n", DXL_BAUDRATE); return 0;
	}

	// Установка скорости порта SERIAL
	if (portHandlerSerial->setBaudRate(SERIAL_BAUDRATE)) {
		fmt::print("SUCCESS: set baudrate to{}\n", SERIAL_BAUDRATE);
	}
	else {
		fmt::print("FAILED: set baudrate to{}\n", SERIAL_BAUDRATE); return 0;
	}

	// Установка скорости сервоприводов 
	for (int _id = 1; _id <= num_servos; _id++) {
		// Запись speed в регистр ADDR_PROFILE_VELOCITY 
		dxl_comm_result = packetHandler->write2ByteTxRx(portHandler, _id, ADDR_PROFILE_VELOCITY, SPEED, &dxl_error);
	}

	// Включение момента сервоприводов
	setTorque(num_servos, TORQUE_ENABLE, packetHandler, portHandler);
	fmt::print("Start joint mode... \n");
	// Регистрация сигнала SIGINT и функции его обработки 
	signal(SIGINT, signalHandler);

	// Пока не нажали Ctrl + C 
	while (!stop) {
		for (int i = 0; i < num_states; i++)
		{
			for (int _id = 1; _id <= num_servos; _id++)
			{
				// Запись целевой позиции для каждого сервопривода из таблицы состояния в регистры ADDR_GOAL_POSITION
				dxl_comm_result = packetHandler->write2ByteTxRx(portHandler, _id, ADDR_GOAL_POSITION, states[i][_id - 1], &dxl_error);

			}
			if (stop) break; sleep(3);

			////////////////////// Включение/отключение насоса
			//////////////////////

			if (i == 3) {
				//включение
				pumpFunction('1');

				fmt::print("PUMP ON!\n"); sleep(2);

			}
			else if (i == 8) {
				//выключение
				pumpFunction('2');

				fmt::print("PUMP OFF\n"); sleep(2);
			}
			// ////////////////////////////////////////////////
		}
	}

	// Перемещение 2 и 3 сервоприводов для складывания манипулятора в стойку:
	uint8_t dxl_index = 2;
	//Запись значения позиции в регистр ADDR_GOAL_POSITION
	packetHandler->write2ByteTxRx(portHandler, dxl_index, ADDR_GOAL_POSITION,START_POS + 250, &dxl_error);
	dxl_index = 3;
	//Запись значения позиции в регистр ADDR_GOAL_POSITION
	packetHandler->write2ByteTxRx(portHandler, dxl_index, ADDR_GOAL_POSITION,START_POS - 900, &dxl_error);

	// Отключение момента
	setTorque(num_servos, TORQUE_DISABLE, packetHandler, portHandler);

	// Закрытие порта 
	portHandler->closePort();
	fmt::print("Closed port\n");
	return 0;
}


