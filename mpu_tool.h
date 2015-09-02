#ifndef __MPU_TOOL_H__
#define __MPU_TOOL_H__



/* Exported define ------------------------------------------------------------*/



/* Exported typedef -----------------------------------------------------------*/



/* Exported variables prototypes ---------------------------------------------*/
/* Data requested by client. */
#define PRINT_ACCEL     (0x01)
#define PRINT_GYRO      (0x02)
#define PRINT_QUAT      (0x04)

#define ACCEL_ON        (0x01)
#define GYRO_ON         (0x02)

#define MOTION          (0)
#define NO_MOTION       (1)

/* Starting sampling rate. */
#define DEFAULT_MPU_HZ  (200)

struct hal_s
{
    unsigned char sensors;
    unsigned char dmp_on;
    volatile unsigned char new_gyro;
    unsigned short report;
    unsigned short dmp_features;
};
static struct hal_s hal;

/* The sensors can be mounted onto the board in any orientation. The mounting
 * matrix seen below tells the MPL how to rotate the raw data from thei
 * driver(s).
 * TODO: The following matrices refer to the configuration on an internal test
 * board at Invensense. If needed, please modify the matrices to match the
 * chip-to-body matrix for your particular set up.
 */
static signed char gyro_orientation[9] = 
{
    -1, 0, 0,
    0, -1, 0,
    0, 0, 1
};

enum packet_type_e
{
    PACKET_TYPE_ACCEL,
    PACKET_TYPE_GYRO,
    PACKET_TYPE_QUAT,
    PACKET_TYPE_TAP,
    PACKET_TYPE_ANDROID_ORIENT,
    PACKET_TYPE_PEDO,
    PACKET_TYPE_MISC
};


/* Exported function prototypes ---------------------------------------------*/

extern void mpu_tool(void);
extern void DMPDriverReport(char* buf, int length, void *data);
extern int  dmp_init(void);
extern void Self_Test(void);
void mpu_tool_2(void);
void Control(void);
#endif


