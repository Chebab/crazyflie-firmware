/**
 *    ||          ____  _ __
 * +------+      / __ )(_) /_______________ _____  ___
 * | 0xBC |     / __  / / __/ ___/ ___/ __ `/_  / / _ \
 * +------+    / /_/ / / /_/ /__/ /  / /_/ / / /_/  __/
 *  ||  ||    /_____/_/\__/\___/_/   \__,_/ /___/\___/
 *
 * Crazyflie control firmware
 *
 * Copyright (C) 2011-2012 Bitcraze AB
 *
 * This program is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation, in version 3.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program. If not, see <http://www.gnu.org/licenses/>.
 *
 * stabilizer.h: Stabilizer orchestrator
 */
#ifndef STABALIZER_H_
#define STABALIZER_H_

#include <stdbool.h>
#include <stdint.h>
#define STATE_SIZE 7
#define INPUT_SIZE 4

extern uint32_t motorPowerM1;  // Motor 1 power output (16bit value used: 0 - 65535)
extern uint32_t motorPowerM2;  // Motor 2 power output (16bit value used: 0 - 65535)
extern uint32_t motorPowerM3;  // Motor 3 power output (16bit value used: 0 - 65535)
extern uint32_t motorPowerM4;  // Motor 4 power output (16bit value used: 0 - 65535)
extern bool isEco;

void stabilizerInit(void);

bool stabilizerTest(void);


#endif /* STABALIZER_H_ */
