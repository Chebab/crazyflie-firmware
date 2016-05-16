#ifndef REFERENCE_GENERATOR_H_
#define REFERENCE_GENERATOR_H_

// unclear what these are used for
#include <stdbool.h>
#include <stdint.h>

extern uint16_t zAccDesired;
extern float eulerRollDesired;   // Measured roll angle in deg
extern float eulerPitchDesired;  // Measured pitch angle in deg
extern float eulerYawDesired;    // Measured yaw angle in deg

void referenceGeneratorInit(void);

bool referenceGeneratorTest(void);


#endif /* REFERENCE_GENERATOR_H_ */
