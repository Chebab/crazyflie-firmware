#ifndef REFERENCE_GENERATOR_H_
#define REFERENCE_GENERATOR_H_

// unclear what these are used for
#include <stdbool.h>
#include <stdint.h>
#include "stabilizer.h"


extern float reference[STATE_SIZE];
void referenceGeneratorInit(void);

bool referenceGeneratorTest(void);


#endif /* REFERENCE_GENERATOR_H_ */
