#pragma once

#include "vector_3.h"

//��Ԫ��
typedef struct
{
	float qw;
	float qx;
	float qy;
	float qz;
}Quaternion;
//��Ч��Ԫ��
//�������м����
//������������Ч��
typedef struct
{
	float qw;
	float qx;
	float qy;
	float qz;
	
	float qw2 , qx2 , qy2, qz2 , qwx , qwy , qwz , qxy , qxz , qyz;
}QuaternionEf;

//�ɱ�׼��Ԫ����ȡ��Ч��Ԫ��
static inline QuaternionEf get_QuaternionEf( Quaternion quat )
{
	QuaternionEf result;
	result.qw = quat.qw;
	result.qx = quat.qx;
	result.qy = quat.qy;
	result.qz = quat.qz;
	result.qw2 = quat.qw * quat.qw;
	result.qx2 = quat.qx * quat.qx;
	result.qy2 = quat.qy * quat.qy;
	result.qz2 = quat.qz * quat.qz;
	result.qwx = quat.qw * quat.qx;
	result.qwy = quat.qw * quat.qy;
	result.qwz = quat.qw * quat.qz;
	result.qxy = quat.qx * quat.qy;
	result.qxz = quat.qx * quat.qz;
	result.qyz = quat.qy * quat.qz;
	return result;
}
static inline Quaternion get_Quaternion( QuaternionEf quat )
{
	Quaternion result;
	result.qw = quat.qw;
	result.qx = quat.qx;
	result.qy = quat.qy;
	result.qz = quat.qz;
	return result;
}

/*��ù�һ����Ԫ��*/
	Quaternion Quaternion_normalize( Quaternion quat );
	QuaternionEf QuaternionEf_normalize( QuaternionEf quat );
/*��ù�һ����Ԫ��*/

/*��ʼ����Ԫ��*/
	static inline Quaternion Quaternion_init_qs( float qw , float qx , float qy , float qz )
	{
		Quaternion quat;
		quat.qw = qw;
		quat.qx = qx;
		quat.qy = qy;
		quat.qz = qz;
		return quat;
	}
/*��ʼ����Ԫ��*/

/*����Ԫ�������ת����*/
	vector3_float Quaternion_get_Rotation_vec( Quaternion quat );
	vector3_float QuaternionEf_get_Rotation_vec( Quaternion quat );
/*����Ԫ�������ת����*/

/*�������Ԫ����ת�������*/
	vector3_float Quaternion_rotate( Quaternion quat , vector3_float vec );
	vector3_float Quaternion_rotate_axis_x( Quaternion quat );
	vector3_float Quaternion_rotate_axis_y( Quaternion quat );
	vector3_float Quaternion_rotate_axis_z( Quaternion quat );
	vector3_float Quaternion_reverse_rotate( Quaternion quat , vector3_float vec );
	vector3_float Quaternion_reverse_rotate_axis_x( Quaternion quat );
	vector3_float Quaternion_reverse_rotate_axis_y( Quaternion quat );
	vector3_float Quaternion_reverse_rotate_axis_z( Quaternion quat );

	vector3_float QuaternionEf_rotate( QuaternionEf quat , vector3_float vec );
	vector3_float QuaternionEf_rotate_axis_x( QuaternionEf quat );
	vector3_float QuaternionEf_rotate_axis_y( QuaternionEf quat );
	vector3_float QuaternionEf_rotate_axis_z( QuaternionEf quat );
	vector3_float QuaternionEf_reverse_rotate( QuaternionEf quat , vector3_float vec );
	vector3_float QuaternionEf_reverse_rotate_axis_x( QuaternionEf quat );
	vector3_float QuaternionEf_reverse_rotate_axis_y( QuaternionEf quat );
	vector3_float QuaternionEf_reverse_rotate_axis_z( QuaternionEf quat );
/*�������Ԫ����ת�������*/

/*��Ԫ������*/
	Quaternion Quaternion_Integral_Runge1( Quaternion quat , vector3_float delta_angle0 );
	Quaternion Quaternion_Integral_Runge2( Quaternion quat , vector3_float delta_angle0 , vector3_float delta_angle1 );
	
	QuaternionEf QuaternionEf_Integral_Runge1( QuaternionEf quat , vector3_float delta_angle0 );
	QuaternionEf QuaternionEf_Integral_Runge2( QuaternionEf quat , vector3_float delta_angle0, vector3_float delta_angle1  );
/*��Ԫ������*/

/*��Ԫ���˷�*/
	Quaternion Quaternion_Mult( Quaternion quat_a , Quaternion quat_b );
	QuaternionEf QuaternionEf_Mult( QuaternionEf quat_a , QuaternionEf quat_b );
/*��Ԫ���˷�*/

/*����ת������ת��Ԫ��*/
	Quaternion Quaternion_rotate_delta_angle( Quaternion quat , vector3_float delta_angle );
	QuaternionEf QuaternionEf_rotate_delta_angle( QuaternionEf quat , vector3_float delta_angle );
/*����ת������ת��Ԫ��*/

/*��Ԫ��תŷ����*/
	float Quaternion_getPitch( Quaternion quat );
	float Quaternion_getRoll( Quaternion quat );
	float Quaternion_getYaw( Quaternion quat );
/*��Ԫ��תŷ����*/

/*����Ԫ������ȡPitch Roll��ת��Ԫ����ȥ��Yaw��*/
	Quaternion Quaternion_get_PRQuat( Quaternion quat );
/*����Ԫ������ȡPitch Roll��ת��Ԫ����ȥ��Yaw��*/

/*��Ԫ��ȡ�棨���*/
	static inline Quaternion Quaternion_conjugate( Quaternion quat )
	{
		Quaternion result;
		result.qw = quat.qw;
		result.qx = -quat.qx;
		result.qy = -quat.qy;
		result.qz = -quat.qz;
		return result;
	}
	static inline QuaternionEf QuaternionEf_conjugate( QuaternionEf quat )
	{
		Quaternion result;
		result.qw = quat.qw;
		result.qx = -quat.qx;
		result.qy = -quat.qy;
		result.qz = -quat.qz;
		return get_QuaternionEf( result );
	}
/*��Ԫ��ȡ�棨���*/