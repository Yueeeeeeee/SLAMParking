#include "datagrams.h"
#include "commands.h"
#include "hal.h"

#include "datatypes.h"
#include "buffer.h"

#include <stdio.h>
#include <string.h>



#define TOTAL_GEAR_RATIO			11.822485208f
#define TIRE_CIRCUMFERENCE_IN_M		0.35908f
#define SEC_PER_MIN					60.0f
#define POLE_COUNT					4

mc_values val;

uint8_t get_fw()
{
	uint8_t command[1] = { COMM_FW_VERSION };
	uint8_t payload[512];
	uint16_t size = 0;

	size = pack(command, 1, payload);
	size = sendData(payload, size);

	for(int i = 0; i < size; i++)
		printf("%d ",payload[i]);
	printf("\n");

	return 0;
}

uint8_t send_alive()
{
	uint8_t command[1] = { COMM_ALIVE };
	uint8_t payload[512];
	uint16_t size = 0;

	size = pack(command, 1, payload);
	sendDataNoReply(payload, size);

	return 0;
}

uint8_t reboot_device()
{
	uint8_t command[1] = { COMM_REBOOT };
	uint8_t payload[512];
	uint16_t size = 0;

	size = pack(command, 1, payload);
	sendDataNoReply(payload, size);

	return 0;
}

// read_values()
// -> private, liest Daten von ECU in globales struct
static uint8_t read_values()
{
	uint8_t command[1] = { COMM_GET_VALUES };
	uint8_t payload[512];
	uint16_t size = 0;

	size = pack(command, 1, payload);
	size = sendData(payload, size);

	int32_t index = 0;
	index++;

	val.temp_fet_filtered[0] = buffer_get_float16(payload, 1e1, &index);
	val.temp_fet_filtered[1] = buffer_get_float16(payload, 1e1, &index);
	val.temp_fet_filtered[2] = buffer_get_float16(payload, 1e1, &index);
	val.temp_fet_filtered[3] = buffer_get_float16(payload, 1e1, &index);
	val.temp_fet_filtered[4] = buffer_get_float16(payload, 1e1, &index);
	val.temp_fet_filtered[5] = buffer_get_float16(payload, 1e1, &index);
	val.temp_motor_filtered = buffer_get_float16(payload, 1e1, &index);
	val.avg_motor_current = buffer_get_float32(payload, 1e2, &index);
	val.avg_input_current = buffer_get_float32(payload, 1e2, &index);
	val.duty_cycle = buffer_get_float16(payload, 1e3, &index);
	val.rpm = buffer_get_float32(payload, 1e0, &index);
	val.input_voltage = buffer_get_float16(payload, 1e1, &index);
	val.amp_hours = buffer_get_float32(payload, 1e4, &index);
	val.amp_hours_charged = buffer_get_float32(payload, 1e4, &index);
	val.watt_hours = buffer_get_float32(payload, 1e4, &index);
	val.watt_hours_charged = buffer_get_float32(payload, 1e4, &index);
	val.tachometer_value = buffer_get_int32(payload, &index);
	val.tachometer_abs_value = buffer_get_int32(payload, &index);
	val.fault_code = (mc_fault_code) payload[index];

	return 0;
}



// get_values(struct values *val)
// -> public, gibt Daten zurück
uint8_t get_values(mc_values *dest_vals)
{
	read_values();
	memcpy(dest_vals, &val, sizeof(*dest_vals));

	return 0;
}

// RPM (motor) to Velocity in m/s
static double rpm_to_velocity(int rpm)
{
    double wheelRpm = rpm / TOTAL_GEAR_RATIO;
	return (wheelRpm * TIRE_CIRCUMFERENCE_IN_M) / ((POLE_COUNT / 2) * SEC_PER_MIN);
}

// Velocity in m/s to RPM (motor)
static int velocity_to_rpm(double velocity)
{
	double wheelRpm = (velocity * (POLE_COUNT / 2) * SEC_PER_MIN) / (TIRE_CIRCUMFERENCE_IN_M);
	return (int) wheelRpm * TOTAL_GEAR_RATIO;
}

 /**
  * @brief Set translatory car velocity in m/s
  *
  * @param v velocity in m/s
  */
void set_velocity(double velocity)
{
	set_rpm(velocity_to_rpm(velocity));
}

 /**
  * @brief Get translatory car velocity in m/s
  *
  * @return velocity in m/s
  */
double get_velocity()
{
	read_values();
	return rpm_to_velocity(val.rpm);
}

 /**
  * @brief Set the rotational speed (RPM) of the motor.
  *
  * @param  rpm rounds per minute
  * @return
  */
uint8_t set_rpm(int rpm)
{
	uint8_t command[512] = { COMM_SET_RPM };
	uint8_t payload[512];
	uint16_t size = 0;
	int32_t index = 0;

	index++;
	buffer_append_int32(command, rpm, &index);

	size = pack(command, 5, payload);
	sendDataNoReply(payload, size);

	return 0;
}

 /**
  * @brief Get the rotational speed (RPM) of the motor.
  *
  * @return rounds per minute
  */
double get_rpm()
{
	read_values();
	return (double) val.rpm;
}