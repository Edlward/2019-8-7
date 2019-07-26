#pragma once

//��ά��������
//20181220 ���Ľ�

typedef unsigned int uint;
typedef unsigned short ushort;

#define vector3( TYPE ) \
typedef struct \
{ \
	TYPE x; \
	TYPE y; \
	TYPE z; \
}vector3_##TYPE

vector3( float );
vector3( double );
vector3( int );
vector3( uint );
vector3( short );
vector3( ushort );

/*�����˱���*/
	#define vector3_mult( TYPE ) \
	vector3_##TYPE vector3_##TYPE##_mult( vector3_##TYPE a , TYPE b ) \
	{ \
		vector3_##TYPE result; \
		result.x = a.x * b; \
		result.y = a.y * b; \
		result.z = a.z * b; \
		return result; \
	}
	static inline vector3_mult( double )
	static inline vector3_mult( float )
	static inline vector3_mult( int )
	static inline vector3_mult( uint )
	static inline vector3_mult( short )
	static inline vector3_mult( ushort )
/*�����˱���*/

/*�������*/
	#define vector3_dot_product( TYPE ) \
	TYPE vector3_##TYPE##_dot_product( vector3_##TYPE a , vector3_##TYPE b ) \
	{ \
		return a.x*b.x + a.y*b.y + a.z*b.z; \
	}	
	static inline vector3_dot_product( double )
	static inline vector3_dot_product( float )
	static inline vector3_dot_product( int )
	static inline vector3_dot_product( uint )
	static inline vector3_dot_product( short )
	static inline vector3_dot_product( ushort )
/*�������*/

/*�������*/
	#define vector3_cross_product( TYPE ) \
	vector3_##TYPE vector3_##TYPE##_cross_product( vector3_##TYPE a , vector3_##TYPE b ) \
	{ \
		vector3_##TYPE result; \
		result.x = a.y*b.z - a.z*b.y; \
		result.y = a.z*b.x - a.x*b.z; \
		result.z = a.x*b.y - a.y*b.x; \
		return result; \
	}
	static inline vector3_cross_product( double )
	static inline vector3_cross_product( float )
	static inline vector3_cross_product( int )
	static inline vector3_cross_product( uint )
	static inline vector3_cross_product( short )
	static inline vector3_cross_product( ushort )
/*�������*/
	
/*����Ԫ�����*/
	#define vector3_elementwise_product( TYPE ) \
	vector3_##TYPE vector3_##TYPE##_elementwise_product( vector3_##TYPE a , vector3_##TYPE b ) \
	{ \
		vector3_##TYPE result; \
		result.x = a.x * b.x; \
		result.y = a.y * b.y; \
		result.z = a.z * b.z; \
		return result; \
	}
	static inline vector3_elementwise_product( double )
	static inline vector3_elementwise_product( float )
	static inline vector3_elementwise_product( int )
	static inline vector3_elementwise_product( uint )
	static inline vector3_elementwise_product( short )
	static inline vector3_elementwise_product( ushort )
/*����Ԫ�����*/
	
/*������*/
	#define vector3_plus( TYPE ) \
	vector3_##TYPE vector3_##TYPE##_plus( vector3_##TYPE a , vector3_##TYPE b ) \
	{ \
		vector3_##TYPE result; \
		result.x = a.x + b.x; \
		result.y = a.y + b.y; \
		result.z = a.z + b.z; \
		return result; \
	}
	static inline vector3_plus( double )
	static inline vector3_plus( float )
	static inline vector3_plus( int )
	static inline vector3_plus( uint )
	static inline vector3_plus( short )
	static inline vector3_plus( ushort )
/*������*/
	
/*������*/
	#define vector3_subtract( TYPE ) \
	vector3_##TYPE vector3_##TYPE##_subtract( vector3_##TYPE a , vector3_##TYPE b ) \
	{ \
		vector3_##TYPE result; \
		result.x = a.x - b.x; \
		result.y = a.y - b.y; \
		result.z = a.z - b.z; \
		return result; \
	}
	static inline vector3_subtract( double )
	static inline vector3_subtract( float )
	static inline vector3_subtract( int )
	static inline vector3_subtract( uint )
	static inline vector3_subtract( short )
	static inline vector3_subtract( ushort )
/*������*/
	
/*����ƽ����*/
	#define vector3_square( TYPE ) \
	TYPE vector3_##TYPE##_square( vector3_##TYPE vec ) \
	{ \
		return vec.x*vec.x + vec.y*vec.y + vec.z*vec.z; \
	}
	static inline vector3_square( double )
	static inline vector3_square( float )
	static inline vector3_square( int )
	static inline vector3_square( uint )
	static inline vector3_square( short )
	static inline vector3_square( ushort )
/*����ƽ����*/
	
/*�����޷�*/
	vector3_float vector3_float_constrain( vector3_float vec , float max_length );
/*�����޷�*/