#ifndef PARSE_H
#define PARSE_H

#define SPLIT_FLOAT_100(value) (int)(value), (int)rint((fabs(value)-(int)(fabs(value)))*100.0)
#define SPLIT_FLOAT(value, precision) (int)(value), (int)rint((fabs(value)-(int)(fabs(value)))*pow(10, precision))

uint32_t digitsToInt(char* command, uint32_t index, uint8_t length, uint8_t base);
uint8_t parseCommand(char* command);

#endif
