/****************************************************************************
 *
 *   Copyright ZHIKUN-TECH co.Ltd. All rights reserved.
 *   Author David.xu
 *   tel 18600837925
 *   Email  xs1119@163.com
 *
 ****************************************************************************/

/**
 * @file batt_serial.cpp
 * Driver angel_batt through a serial port
 */

#include <nuttx/clock.h>
#include <sys/types.h>
#include <stdint.h>
#include <stdio.h>
#include <stdbool.h>
#include <stdlib.h>
#include <string.h>
#include <fcntl.h>
#include <poll.h>
#include <errno.h>
#include <stdio.h>
#include <math.h>
#include <unistd.h>
#include <fcntl.h>
#include <px4_config.h>
#include <nuttx/arch.h>
#include <arch/board/board.h>
#include <drivers/drv_hrt.h>
#include <drivers/device/i2c.h>


#include <uORB/uORB.h>
#include <uORB/topics/battery_status.h>
#include <termios.h>
#include <errno.h>
#include <systemlib/err.h>
#include <drivers/drv_hrt.h>

#include <board_config.h>
#include "qiaoliang/qiaoliang_define.h"
//#include "gps_helper.h"

#define BATT_SERIAL_DEVICE_PATH	"/dev/batt_serial"
#define BATT_SERIAL_UART_PORT "/dev/ttyS2"

#define MAX_OUT 3
#define DELTA_VOL 6.0f

#define PROTOCOL_NUM 30
#define BUFF_NUM 10
#define SEND_NUM 3

//static float value_buf_0[MAX_OUT];
//static uint8_t count_0=0;
/* oddly, ERROR is not defined for c++ */
#ifdef ERROR
# undef ERROR
#endif
static const int ERROR = -1;

/* class for dynamic allocation of satellite info data */

class BATT_SERIAL : public device::CDev
{
public:
	BATT_SERIAL(const char *uart_path);
	virtual ~BATT_SERIAL();

	virtual int			init();

//	virtual int			ioctl(struct file *filp, int cmd, unsigned long arg);

	/**
	 * Diagnostics - print some basic information about the driver.
	 */
	//void				print_info();

private:

	bool				_task_should_exit;				///< flag to make the main worker task exit
	int					_serial_fd;					///< serial interface to GPS
	unsigned				_baudrate;					///< current baudrate
	volatile int		_task;						///< worker task
	uint8_t 			_rece_buff[PROTOCOL_NUM+BUFF_NUM];
	uint8_t 			_send_buff[SEND_NUM];
	float 				_voltage;
	float				_voltage_filter;
	float				_current;
	float				_power;
	float 				_remain_capacity;
	float 				_tempature;
	float 				_voltage_1;
	float 				_voltage_2;
	float 				_voltage_3;
	float 				_voltage_4;
	float 				_voltage_5;
	float 				_voltage_6;
	float 				_voltage_7;
	float 				_voltage_8;
	float 				_crit_thr;
	float				_low_thr;
	float 				_emergency_thr;
	uint16_t 			_charge_battery;
	uint8_t 			_batt_type;
	uint8_t 			_system_status;
	uint8_t 			_cycle_count;
	uint8_t 			_error;
	uint8_t 			_dcm;
	uint8_t 			_resis_ud;
	
	uint64_t				_now;
	uint64_t				_battery_last_timestamp;
	float				_battery_mamphour_total;
	bool				_startbyte;
	int 					_byteindex;

	struct battery_status_s	_report_battery_status;				///< uORB topic for gps position
	orb_advert_t			_report_battery_status_pub;				///< uORB pub for gps position


	/**
	 * Trampoline to the worker task
	 */
	static void			task_main_trampoline(void *arg);


	/**
	 * Worker task: main GPS thread that configures the GPS and parses incoming data, always running
	 */
	void				task_main(void);
	int 				set_baudrate(const int &fd, unsigned baud);
	float 	_filter(float value,float *value_buf);


};


/*
 * Driver 'main' command.
 */
extern "C" __EXPORT int batt_smbus_serial_main(int argc, char *argv[]);

namespace
{

BATT_SERIAL *batt_serial_dev = nullptr;

}

BATT_SERIAL::BATT_SERIAL(const char *uart_path) :
	CDev("batt_serial", BATT_SERIAL_DEVICE_PATH),
	_task_should_exit(false),
	_serial_fd(-1),
	_baudrate(57600),
	_task(-1),
	_voltage(0),
	_voltage_filter(0),
	_current(0),
	_power(0),
	_remain_capacity(0),
	_tempature(0),
	_voltage_1(0),
	_voltage_2(0),
	_voltage_3(0),
	_voltage_4(0),
	_voltage_5(0),
	_voltage_6(0),
	_voltage_7(0),
	_voltage_8(0),
	_crit_thr(0),
	_low_thr(0),
	_emergency_thr(0),
	_charge_battery(0),
	_batt_type(0),
	_system_status(0),
	_cycle_count(0),
	_error(0),
	_now(0),
	_battery_last_timestamp(0),
	_battery_mamphour_total(0),
	_startbyte(false),
	_byteindex(0),
	_report_battery_status_pub(nullptr)
	
{
	batt_serial_dev = this;
	memset(&_report_battery_status, 0, sizeof(_report_battery_status));
	memset(_rece_buff,0,sizeof(_rece_buff));
	memset(_send_buff,0,sizeof(_send_buff));
	_report_battery_status_pub = nullptr;

}

BATT_SERIAL::~BATT_SERIAL()
{
	/* tell the task we want it to go away */
	_task_should_exit = true;
	/* spin waiting for the task to stop */
	for (unsigned i = 0; (i < 10) && (_task != -1); i++) {
		/* give it another 100ms */
		usleep(100000);
	}

	/* well, kill it anyway, though this will probably crash */
	if (_task != -1) {
		task_delete(_task);
	}

	batt_serial_dev = nullptr;


}

int
BATT_SERIAL::init()
{
	int ret = ERROR;

//	printf("BATT_SERIAL Init\r\n");

	/* do regular cdev init */
	if (CDev::init() != OK) {
		goto out;
	}

	/* start the GPS driver worker task */
	_task = px4_task_spawn_cmd("batt_serial", SCHED_DEFAULT,
				   SCHED_PRIORITY_SLOW_DRIVER, 1200, (main_t)&BATT_SERIAL::task_main_trampoline, nullptr);

	if (_task < 0) {
		warnx("task start failed: %d", errno);
		return -errno;
	}

	ret = OK;
out:
	return ret;
}

/*
int
BATT_SERIAL::ioctl(struct file *filp, int cmd, unsigned long arg)
{
	lock();

	int ret = OK;

	switch (cmd) {

	default:
		give it to parent if no one wants it 
		ret = CDev::ioctl(filp, cmd, arg);
		break;
	}

	unlock();

	return ret;
}
*/


void
BATT_SERIAL::task_main_trampoline(void *arg)
{
	batt_serial_dev->task_main();
}

void
BATT_SERIAL::task_main()
{
	pollfd fds[1];
	/* open the serial port */
	_serial_fd = ::open(BATT_SERIAL_UART_PORT, O_RDWR);

	//PX4_INFO("BATT_SERIAL  _serial_fd  %d",_serial_fd);
	unsigned baud_batt_serial =57600;	

	uint8_t tmp_buff[PROTOCOL_NUM+BUFF_NUM];
	memset(tmp_buff,0,sizeof(tmp_buff));
	
	if (_serial_fd < 0) {
		DEVICE_LOG("failed to open serial port: %s err: %d", BATT_SERIAL_UART_PORT, errno);
		/* tell the dtor that we are exiting, set error code */
		_task = -1;
		_exit(1);
	}

	_send_buff[0]=0xaa;
	_send_buff[1]=0x01;
	_send_buff[2]=0x55;	
	
	set_baudrate(_serial_fd,baud_batt_serial);


	fds[0].fd = _serial_fd;
	fds[0].events = POLLIN;


	if (_crit_thr < 0.01f) {
		param_get(param_find("BAT_CRIT_THR"), &_crit_thr);
	}

	if (_low_thr < 0.01f) {
		param_get(param_find("BAT_LOW_THR"), &_low_thr);
	}

	if (_emergency_thr < 0.01f) {
		param_get(param_find("BAT_EMERGEN_THR"), &_emergency_thr);
	}
//	PX4_INFO("_crit_thr %.3f _low_thr %.3f _emergency_thr %.3f",(double)_crit_thr,(double)_low_thr,(double)_emergency_thr);

	/* loop handling received serial bytes and also configuring in between */
	while (!_task_should_exit) {

		int ret2 = ::write(_serial_fd,_send_buff,sizeof(_send_buff));
        if(ret2==3){}
		/* then poll for new data */
		int ret = ::poll(fds, sizeof(fds) / sizeof(fds[0]), 500);
		//PX4_INFO("BATT_SERIAL poll ret  %d",ret);
		if (ret < 0) {
			/* something went wrong when polling */
			usleep(500*1000);
			continue;

		} else if (ret == 0) {
			/* Timeout */
			continue;

		} else if (ret > 0) {
			if (fds[0].revents & POLLIN) {

				    memset(tmp_buff,0,sizeof(tmp_buff));
    				int count = ::read(_serial_fd, tmp_buff, sizeof(tmp_buff));
					//PX4FLOW_WARNX((nullptr,"--read_buf_count %d",count));

					char ch;
    				if (count > 0 )
    				{
    					for (int i = 0; i < count;i++)
    					{
	    					ch = tmp_buff[i];
							//PX4FLOW_WARNX((nullptr,"parse char %x",ch));
	    					switch (ch)
	    					{
	    						case 0xaa:
									_rece_buff[_byteindex] = ch;
									
									if (!_startbyte)
									{
		    							_startbyte = true;
		    							_byteindex = 1;
									}
									else{

	    								_byteindex++;
										if (_byteindex == PROTOCOL_NUM-1)
											_startbyte = false;
									}
	    							break;
	    						case 0x55:
									_rece_buff[_byteindex] = ch;
									if (_startbyte)
									{
	    								_byteindex++;
										if (_byteindex == PROTOCOL_NUM-1)
										{
											_startbyte = false;
										}
									}else{
	                 		           _byteindex = 0;
	                        			if ((uint8_t)_rece_buff[0]== 0xaa &&(uint8_t)_rece_buff[PROTOCOL_NUM-1] == 0x55)
	                        			{
											short tmp,tmp1;
											tmp=(((_rece_buff[1]<<8)&0xFF00))|(_rece_buff[2]&0xFF);	
											_voltage=tmp*0.00125f;
											
											tmp=(((_rece_buff[3]<<8)&0xFF00))|(_rece_buff[4]&0xFF);	
											if(tmp<65535/2){
												_current = tmp*0.005f;
											}else{
												_current = (65535-tmp)*0.005f;
											}
											//tmp=(((_rece_buff[5]<<8)&0xFF00))|(_rece_buff[6]&0xFF);
//											_power=tmp*0.125f;



											tmp=(((_rece_buff[7]<<8)&0xFF00))|(_rece_buff[8]&0xFF);
											_remain_capacity=tmp/1.0f;
											tmp=(((_rece_buff[9]<<8)&0xFF00))|(_rece_buff[10]&0xFF);
											_tempature=tmp/10.0f;
											tmp=_rece_buff[11]&0xFF;
											_voltage_1=tmp/10.0f;
											tmp=_rece_buff[12]&0xFF;
											_voltage_2=tmp/10.0f;
											tmp=_rece_buff[13]&0xFF;
											_voltage_3=tmp/10.0f;
											tmp=_rece_buff[14]&0xFF;
											_voltage_4=tmp/10.0f;
											tmp=_rece_buff[15]&0xFF;
											_voltage_5=tmp/10.0f;
											tmp=_rece_buff[16]&0xFF;
											_voltage_6=tmp/10.0f;
											tmp=_rece_buff[17]&0xFF;
											_voltage_7=tmp/10.0f;
											tmp=_rece_buff[18]&0xFF;
											_voltage_8=tmp/10.0f;
											tmp=_rece_buff[19]&0xFF;
											tmp1=_rece_buff[20]&0xFF;
											_charge_battery = (tmp<<8)|tmp1;
											tmp=_rece_buff[21]&0xFF;
											_batt_type=tmp;
											tmp=_rece_buff[22]&0xFF;
											_system_status=tmp;
											tmp=_rece_buff[23]&0xFF;
											_cycle_count=tmp;
											tmp=_rece_buff[24]&0xFF;
											_error=tmp;
											tmp=_rece_buff[25]&0xFF;
											_dcm=tmp;
											tmp=_rece_buff[26]&0xFF;
											_resis_ud=tmp;
											
/*											_voltage_filter = _filter(_voltage,value_buf_0);
												if(fabsf(_voltage-_voltage_filter)>DELTA_VOL){
													count_0++;
													_report_battery_status.voltage_v =value_buf_0[0];
													_report_battery_status.voltage_filtered_v = value_buf_0[0];
													if(count_0>MAX_OUT){
														for(int i=0;i<MAX_OUT;i++){
															value_buf_0[i]=_voltage;
														}
														count_0 = 0;
													}
												}else{
													count_0 = 0;
													

												//}
*/											
											_now = hrt_absolute_time();
											_power = _voltage*_current;

											_report_battery_status.timestamp = _now;
											_report_battery_status.voltage_v = _voltage;
											_report_battery_status.voltage_filtered_v = _voltage;
											_report_battery_status.current_a = _current;
//											_report_battery_status.charge_battery = _charge_battery;
										
//											PX4_INFO("_voltage--%.2f current %.2f _power %.2f _voltage_1-6 %.2f %.2f %.2f %.2f %.2f %.2f _power_type %d _error %d _dcm %d _resis_ud %d",
											//	(double)_voltage,(double)_current,(double)((_voltage*_current)/_power),(double)_voltage_1,
											//	(double)_voltage_2,(double)_voltage_3,(double)_voltage_4,(double)_voltage_5,
											//	(double)_voltage_6,_power_type,_error,_dcm,_resis_ud));
											if (_battery_last_timestamp != 0) 
											{
												_battery_mamphour_total += _current *(_now- _battery_last_timestamp) * 1.0e-3f / 3600;
											}
											_report_battery_status.discharged_mah = _battery_mamphour_total;
											_battery_last_timestamp = _now;
											_report_battery_status.power= _power;
											_report_battery_status.voltage_v1 = _voltage_1;
											_report_battery_status.voltage_v2 = _voltage_2;
											_report_battery_status.voltage_v3 = _voltage_3;
											_report_battery_status.voltage_v4 = _voltage_5;
											_report_battery_status.voltage_v5 = _voltage_5;
											_report_battery_status.voltage_v6 = _voltage_6;
											_report_battery_status.cycle_count = _cycle_count;
											_report_battery_status.battery_status = _batt_type;

											_report_battery_status.error_status = _error;
											_report_battery_status.system_status = _system_status;
											_report_battery_status.dcm_status = _dcm;
											_report_battery_status.res_status = _resis_ud;											

											//this is to be prove correct;
											_report_battery_status.connected = _voltage>1.0f;
											_report_battery_status.system_source = 1;
											_report_battery_status.priority=1;


											if (_voltage_1 > _low_thr) {
												_report_battery_status.warning = battery_status_s::BATTERY_WARNING_NONE;
											
											} else if (_voltage_1 > _crit_thr) {
												_report_battery_status.warning = battery_status_s::BATTERY_WARNING_LOW;
											
											} else if (_voltage_1 > _emergency_thr) {
												_report_battery_status.warning = battery_status_s::BATTERY_WARNING_CRITICAL;
											
											} else {
												_report_battery_status.warning = battery_status_s::BATTERY_WARNING_EMERGENCY;
											}

											PX4_ZK("power %.7f _batt_type %d",(double)_power,_batt_type);

				                            if(_report_battery_status_pub !=nullptr){
											

												//PX4FLOW_WARNX((nullptr,"_voltage--%.2f current %.2f _power %.2f _voltage_1-6 %.2f %.2f %.2f %.2f %.2f %.2f _power_type %d _error %d _dcm %d _resis_ud %d",
											//	(double)_voltage,(double)_current,(double)((_voltage*_current)/_power),(double)_voltage_1,
											//	(double)_voltage_2,(double)_voltage_3,(double)_voltage_4,(double)_voltage_5,
											//	(double)_voltage_6,_power_type,_error,_dcm,_resis_ud));
												orb_publish(ORB_ID(battery_status), _report_battery_status_pub, &_report_battery_status);

											}else{
												_report_battery_status_pub= orb_advertise(ORB_ID(battery_status), &_report_battery_status);

											}
	                            			memset(_rece_buff,0,sizeof(_rece_buff));
	                        				}
										}
								    usleep(100*1000);
	    							break;
								
	    						default:
								_rece_buff[_byteindex] = ch;
	    						if (_startbyte)
	    						{
	    								_byteindex++;
										if (_byteindex == PROTOCOL_NUM-1)
											_startbyte = false;
//    							PX4FLOW_WARNX((nullptr,"-----read char:%x",ch));
	    						}
								else
								{
//						    	  PX4FLOW_WARNX((nullptr,"-----stop bit error,0x55 != %x",ch));
										_byteindex = 0;
								}
	    							break;
	    					}
    					}
    				}else{
						    usleep(500*1000);
							continue;
					}
			}
		}


		}

	warnx("batt_serial exiting");

	::close(_serial_fd);

	/* tell the dtor that we are exiting */
	_task = -1;
	_exit(0);
	
}

int
BATT_SERIAL::set_baudrate(const int &fd, unsigned baud)
{
	/* process baud rate */
	int speed;

	switch (baud) {

	case 57600:  speed = B57600;  break;

	case 115200: speed = B115200; break;

	default:
		warnx("ERR: baudrate: %d\n", baud);
		return -EINVAL;
	}

	struct termios uart_config;

	int termios_state;

	/* fill the struct for the new configuration */
	tcgetattr(fd, &uart_config);

	/* clear ONLCR flag (which appends a CR for every LF) */
	uart_config.c_oflag &= ~ONLCR;
	/* no parity, one stop bit */
	uart_config.c_cflag &= ~(CSTOPB | PARENB);

	/* set baud rate */
	if ((termios_state = cfsetispeed(&uart_config, speed)) < 0) {
		warnx("ERR: %d (cfsetispeed)\n", termios_state);
		return -1;
	}

	if ((termios_state = cfsetospeed(&uart_config, speed)) < 0) {
		warnx("ERR: %d (cfsetospeed)\n", termios_state);
		return -1;
	}

	if ((termios_state = tcsetattr(fd, TCSANOW, &uart_config)) < 0) {
		warnx("ERR: %d (tcsetattr)\n", termios_state);
		return -1;
	}

	return 0;
}
float
BATT_SERIAL::_filter(float value,float *value_buf){

	float sum = value;

	for (int i=0;i<MAX_OUT - 1;i++){
		value_buf[i]=value_buf[i+1];
		sum += value_buf[i];
	}
	value_buf[MAX_OUT-1] = value;
	return (float)(sum/(MAX_OUT));

}


/**
 * Local functions in support of the shell command.
 */
namespace batt_serial
{

BATT_SERIAL	*batt_serial_dev = nullptr;

void	start();
void	stop();
void	test();
void	reset();
void	info();

/**
 * Start the driver.
 */
void
start()
{
	//int fd;

	if (batt_serial_dev != nullptr) {
		errx(1, "already started");
	}

	/* create the driver */
	batt_serial_dev = new BATT_SERIAL(BATT_SERIAL_UART_PORT);

	if (batt_serial_dev == nullptr) {
		goto fail;
	}

	if (OK != batt_serial_dev->init()) {
		goto fail;
	}

	/* set the poll rate to default, starts automatic data collection */
	/*fd = open(BATT_SERIAL_UART_PORT, O_RDONLY);

	if (fd < 0) {
		errx(1, "open: %s\n", BATT_SERIAL_DEVICE_PATH);
		goto fail;
	}*/

	exit(0);

fail:

	if (batt_serial_dev != nullptr) {
		delete batt_serial_dev;
		batt_serial_dev = nullptr;
	}

	errx(1, "start failed");
}

/**
 * Stop the driver.
 */
void
stop()
{
	delete batt_serial_dev;
	batt_serial_dev = nullptr;

	exit(0);
}

/**
 * Perform some basic functional tests on the driver;
 * make sure we can collect data from the sensor in polled
 * and automatic modes.
 */
void
test()
{

	errx(0, "PASS");
}

/**
 * Reset the driver.
 */
void
reset()
{
	int fd = open(BATT_SERIAL_UART_PORT, O_RDONLY);

	if (fd < 0) {
		err(1, "failed ");
	}


	exit(0);
}

/**
 * Print the status of the driver.
 */
void
info()
{
	if (batt_serial_dev == nullptr) {
		errx(1, "not running");
	}

//	batt_serial_dev->print_info();

	exit(0);
}

} // namespace


int
batt_smbus_serial_main(int argc, char *argv[])
{

	/* set to default */

	/*
	 * Start/load the driver.
	 */
	if (!strcmp(argv[1], "start")) {
		batt_serial::start();
	}

	if (!strcmp(argv[1], "stop")) {
		batt_serial::stop();
	}

	/*
	 * Test the driver/device.
	 */
	if (!strcmp(argv[1], "test")) {
		batt_serial::test();
	}

	/*
	 * Reset the driver.
	 */
	if (!strcmp(argv[1], "reset")) {
		batt_serial::reset();
	}

	/*
	 * Print driver status.
	 */
	if (!strcmp(argv[1], "status")) {
		batt_serial::info();
	}
	errx(1, "unrecognized command, try 'start', 'stop', 'test', 'reset' or 'status'\n [-d /dev/ttyS0-n][-f (for enabling fake)][-s (to enable sat info)]");
}

