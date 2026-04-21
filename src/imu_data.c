/*
* 根据不同的平台和不同的驱动包含不同的头文件
**************************************************/
//#include "xxx.h"
#include <stddef.h>
#include <string.h>
#include "imu_data.h"

/*------------------------------------------------MARCOS define------------------------------------------------*/
#define PROTOCOL_FIRST_BYTE_POS 		0
#define PROTOCOL_SECOND_BYTE_POS		1

#define PROTOCOL_TID_LEN				2
#define PROTOCOL_MIN_LEN				7	/*header(2B) + tid(2B) + len(1B) + CK1(1B) + CK2(1B)*/

#define CRC_CALC_START_POS				2
#define CRC_CALC_LEN(payload_len)		((payload_len) + 3)	/*3 = tid(2B) + len(1B)*/
#define PROTOCOL_CRC_DATA_POS(payload_len)			(CRC_CALC_START_POS + CRC_CALC_LEN(payload_len))

#define PAYLOAD_POS						5

#define SINGLE_DATA_BYTES				4

/*data id define*/
#define SENSOR_TEMP_ID			(unsigned char)0x01
#define ACCEL_ID				(unsigned char)0x10
#define ANGLE_ID				(unsigned char)0x20
#define MAGNETIC_ID				(unsigned char)0x30     /*归一化值*/
#define RAW_MAGNETIC_ID			(unsigned char)0x31     /*原始值*/
#define EULER_ID				(unsigned char)0x40
#define QUATERNION_ID			(unsigned char)0x41
#define UTC_ID					(unsigned char)0x50
#define SAMPLE_TIMESTAMP_ID		(unsigned char)0x51
#define DATA_READY_TIMESTAMP_ID	(unsigned char)0x52
#define LOCATION_ID				(unsigned char)0x60		/*普通精度位置，米级*/
#define HRES_LOCATION_ID		(unsigned char)0x68		/*高精度位置，厘米级*/
#define SPEED_ID				(unsigned char)0x70
#define NAV_STATUS_ID			(unsigned char)0x80

/*length for specific data id*/
#define SENSOR_TEMP_DATA_LEN			(unsigned char)0x2
#define ACCEL_DATA_LEN					(unsigned char)12
#define ANGLE_DATA_LEN					(unsigned char)12
#define MAGNETIC_DATA_LEN				(unsigned char)12
#define MAGNETIC_RAW_DATA_LEN			(unsigned char)12
#define EULER_DATA_LEN					(unsigned char)12
#define QUATERNION_DATA_LEN				(unsigned char)16
#define UTC_DATA_LEN					(unsigned char)11
#define SAMPLE_TIMESTAMP_DATA_LEN		(unsigned char)4
#define DATA_READY_TIMESTAMP_DATA_LEN	(unsigned char)4
#define LOCATION_DATA_LEN				(unsigned char)12
#define HRES_LOCATION_DATA_LEN			(unsigned char)20
#define SPEED_DATA_LEN          		(unsigned char)12
#define NAV_STATUS_DATA_LEN				(unsigned char)1

/*factor for sensor data*/
#define NOT_MAG_DATA_FACTOR			0.000001f
#define MAG_RAW_DATA_FACTOR			0.001f
#define SENSOR_TEMP_DATA_FACTOR		0.01f

/*factor for gnss data*/
#define HRES_LONG_LAT_DATA_FACTOR	0.0000000001
#define LONG_LAT_DATA_FACTOR		0.0000001
#define ALT_DATA_FACTOR				0.001f
#define SPEED_DATA_FACTOR			0.001f

/*------------------------------------------------Variables define------------------------------------------------*/

/*------------------------------------------------Functions declare-----------------------------------------------*/
short get_signed_short(unsigned char *data);
int get_signed_int(unsigned char *data);
int calc_checksum(unsigned char *data, unsigned short len, unsigned short *checksum);

/*-------------------------------------------------------------------------------------------------------------*/
int check_data_len_by_id(payload_data_t header, unsigned char *data, protocol_info_t *info)
{
	int ret = analysis_done; // 默认返回已处理

	if(NULL == data || (unsigned char)0 == header.data_len || NULL == info)
	{
		return para_err;
	}

	switch(header.data_id)
	{
		case SENSOR_TEMP_ID:
		{
			if(SENSOR_TEMP_DATA_LEN == header.data_len)
			{
				ret = analysis_ok;
				info->sensor_temp = get_signed_short(data) * SENSOR_TEMP_DATA_FACTOR;
			}
			else { ret = data_len_err; }
		}
		break;

		case ACCEL_ID:
		{
			if(ACCEL_DATA_LEN == header.data_len)
			{
				ret = analysis_ok;
				info->accel.x = get_signed_int(data) * NOT_MAG_DATA_FACTOR;
				info->accel.y = get_signed_int(data + SINGLE_DATA_BYTES) * NOT_MAG_DATA_FACTOR;
				info->accel.z = get_signed_int(data + SINGLE_DATA_BYTES * 2) * NOT_MAG_DATA_FACTOR;
			}
			else { ret = data_len_err; }
		}
		break;

		case ANGLE_ID:
		{
			if(ANGLE_DATA_LEN == header.data_len)
			{
				ret = analysis_ok;
				info->angle_rate.x = get_signed_int(data) * NOT_MAG_DATA_FACTOR;
				info->angle_rate.y = get_signed_int(data + SINGLE_DATA_BYTES) * NOT_MAG_DATA_FACTOR;
				info->angle_rate.z = get_signed_int(data + SINGLE_DATA_BYTES * 2) * NOT_MAG_DATA_FACTOR;
			}
			else { ret = data_len_err; }
		}
		break;

		case MAGNETIC_ID:
		case RAW_MAGNETIC_ID:
		{
			if(MAGNETIC_DATA_LEN == header.data_len)
			{
				ret = analysis_ok;
				float factor = (header.data_id == MAGNETIC_ID) ? NOT_MAG_DATA_FACTOR : MAG_RAW_DATA_FACTOR;
				axis_data_t *target = (header.data_id == MAGNETIC_ID) ? &info->norm_mag : &info->raw_mag;
				
				target->x = get_signed_int(data) * factor;
				target->y = get_signed_int(data + SINGLE_DATA_BYTES) * factor;
				target->z = get_signed_int(data + SINGLE_DATA_BYTES * 2) * factor;
			}
			else { ret = data_len_err; }
		}
		break;

		case EULER_ID:
		{
			if(EULER_DATA_LEN == header.data_len)
			{
				ret = analysis_ok;
				info->attitude.pitch = get_signed_int(data) * NOT_MAG_DATA_FACTOR;
				info->attitude.roll = get_signed_int(data + SINGLE_DATA_BYTES) * NOT_MAG_DATA_FACTOR;
				info->attitude.yaw = get_signed_int(data + SINGLE_DATA_BYTES * 2) * NOT_MAG_DATA_FACTOR;
			}
			else { ret = data_len_err; }
		}
		break;

		case QUATERNION_ID:
		{
			if(QUATERNION_DATA_LEN == header.data_len)
			{
				ret = analysis_ok;
				info->attitude.quaternion_data0 = get_signed_int(data) * NOT_MAG_DATA_FACTOR;
				info->attitude.quaternion_data1 = get_signed_int(data + SINGLE_DATA_BYTES) * NOT_MAG_DATA_FACTOR;
				info->attitude.quaternion_data2 = get_signed_int(data + SINGLE_DATA_BYTES * 2) * NOT_MAG_DATA_FACTOR;
				info->attitude.quaternion_data3 = get_signed_int(data + SINGLE_DATA_BYTES * 3) * NOT_MAG_DATA_FACTOR;
			}
			else { ret = data_len_err; }
		}
		break;

		case LOCATION_ID:
		{
			if(LOCATION_DATA_LEN == header.data_len)
			{
				ret = analysis_ok;
				info->location.latitude  = get_signed_int(data) * LONG_LAT_DATA_FACTOR;
				info->location.longtidue = get_signed_int(data + SINGLE_DATA_BYTES) * LONG_LAT_DATA_FACTOR;
				info->location.altidue 	 = get_signed_int(data + SINGLE_DATA_BYTES * 2)  * ALT_DATA_FACTOR;
			}
			else { ret = data_len_err; }
		}
		break;

		case HRES_LOCATION_ID:
		{
			if(HRES_LOCATION_DATA_LEN == header.data_len)
			{
				ret = analysis_ok;
				long long temp_lat = 0, temp_lon = 0;
				// 使用 memcpy 解决对齐取值问题
				memcpy(&temp_lat, data, 8);
				memcpy(&temp_lon, data + 8, 8);
				info->location.latitude  = temp_lat * HRES_LONG_LAT_DATA_FACTOR;
				info->location.longtidue = temp_lon * HRES_LONG_LAT_DATA_FACTOR;
				info->location.altidue 	 = get_signed_int(data + 16)  * ALT_DATA_FACTOR;
			}
			else { ret = data_len_err; }
		}
		break;

		case SPEED_ID:
		{
			if(SPEED_DATA_LEN == header.data_len)
			{
				ret = analysis_ok;
				info->vel.vel_n = get_signed_int(data) * SPEED_DATA_FACTOR;
				info->vel.vel_e = get_signed_int(data + SINGLE_DATA_BYTES) * SPEED_DATA_FACTOR;
				info->vel.vel_d = get_signed_int(data + SINGLE_DATA_BYTES * 2) * SPEED_DATA_FACTOR;
			}
			else { ret = data_len_err; }
		}
		break;

		case UTC_ID:
		{
			if(UTC_DATA_LEN == header.data_len)
			{
				ret = analysis_ok;
				// 修复：拷贝到结构体首地址而非 msecond 成员地址，避免偏移错误
				memcpy((unsigned char *)&info->utc, data, sizeof(utc_time_t));
			}
			else { ret = data_len_err; }
		}
		break;

		case NAV_STATUS_ID:
		{
			if(NAV_STATUS_DATA_LEN == header.data_len)
			{
				ret = analysis_ok;
				nav_status_t status;
				memcpy(&status, data, 1);
				info->status = status;
			}
			else { ret = data_len_err; }
		}
		break;

		case SAMPLE_TIMESTAMP_ID:
		case DATA_READY_TIMESTAMP_ID:
		{
			if(SAMPLE_TIMESTAMP_DATA_LEN == header.data_len)
			{
				ret = analysis_ok;
				unsigned int ts = 0;
				memcpy(&ts, data, 4);
				if(header.data_id == SAMPLE_TIMESTAMP_ID)
					info->sample_timestamp = ts;
				else
					info->data_ready_timestamp = ts;
			}
			else { ret = data_len_err; }
		}
		break;

		default:
			ret = analysis_done;
		break;
	}

	return ret;
}

int analysis_data(unsigned char *data, short len, protocol_info_t *info)
{
	unsigned short payload_len = 0;
	unsigned short check_sum_calc = 0;
	unsigned short check_sum_recv = 0;
	unsigned short pos = 0;
	int ret = analysis_done;

	output_data_header_t *header = NULL;
	payload_data_t *payload = NULL;

	if(NULL == data || len < PROTOCOL_MIN_LEN)
	{
		return data_len_err;
	}

	/* 判断帧头 */
	if(PROTOCOL_FIRST_BYTE == data[PROTOCOL_FIRST_BYTE_POS] && \
		PROTOCOL_SECOND_BYTE == data[PROTOCOL_SECOND_BYTE_POS])
	{
		header = (output_data_header_t *)data;
		payload_len = header->len;

		/* 增加一层保护，防止长度字节错误导致内存溢出 */
		if(payload_len > 250) return analysis_done; 

		if(payload_len + PROTOCOL_MIN_LEN > len)
		{
			return 	data_len_err;
		}

		/* 校验计算 */
		calc_checksum(data + CRC_CALC_START_POS, CRC_CALC_LEN(payload_len), &check_sum_calc);
		
		/* 修复：使用 memcpy 获取接收到的校验和，防止对齐问题 */
		memcpy(&check_sum_recv, data + PROTOCOL_CRC_DATA_POS(payload_len), 2);
		
		if(check_sum_calc != check_sum_recv)
		{
			return crc_err;
		}

		/* 解析 Payload */
		pos = PAYLOAD_POS;
		int remaining_payload = payload_len;

		while(remaining_payload > (int)sizeof(payload_data_t))
		{
			payload = (payload_data_t *)(data + pos);
			
			// 再次检查长度，防止非法 payload 导致死循环
			if(payload->data_len + (int)sizeof(payload_data_t) > remaining_payload) break;

			ret = check_data_len_by_id(*payload, (unsigned char *)payload + sizeof(payload_data_t), info);
			
			int step = payload->data_len + sizeof(payload_data_t);
			pos += step;
			remaining_payload -= step;
		}

		return analysis_ok;
	}
	else
	{
		return analysis_done;
	}
}



/*------------------------------------------------------------------------------------------*/
int calc_checksum(unsigned char *data, unsigned short len, unsigned short *checksum)
{
	unsigned char check_a = 0;	/*数据类型为char，在不为char的时候，某些情况下crc计算时错误的*/
	unsigned char check_b = 0;
	unsigned short i;

	if(NULL == data || 0 == len || NULL == checksum)
	{
		return para_err;
	}

	for(i = 0; i < len; i++)
	{
		check_a += data[i];
		check_b += check_a;
	}

	*checksum = ((unsigned short)(check_b << 8) | check_a);

	return analysis_ok;
}

short get_signed_short(unsigned char *data)
{
	short temp = 0;

	temp = (short)((data[1] << 8) | data[0]);

	return temp;
}

int get_signed_int(unsigned char *data)
{
	int temp = 0;

	temp = (int)((data[3] << 24) | (data[2] << 16) | (data[1] << 8) | data[0]);

	return temp;
}
