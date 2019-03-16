/**
 * 
BLE scan sous alarm: mix entre epoll/timerfd et bt natif (HAL)
* 
compilation: mettre dans un dossier dans system/bt/ par exemple:
system/bt/vvnx_alrm
et ajouter ce dir à la liste des subdirs de system/bt/Android.bp

adb push out/target/product/mido/system/bin/bt_alrm /system/bin

démarrage par init en service:

/etc/init/bt_alrm.rc (chmod 644)
	service bt_alrm /system/bin/bt_alrm
	    class main
    
 
chcon u:object_r:bluetoothtbd_exec:s0 /system/bin/bt_alrm
chmod 755 /system/bin/bt_alrm

*/

#include <errno.h>
#include <unistd.h>
#include <stdio.h>
#include <fcntl.h>
#include <stdlib.h>
#include <string.h>
#include <cutils/klog.h>
#include <time.h>
#include <sys/epoll.h> 
#include <sys/timerfd.h>

#include "btcore/include/hal_util.h"
#include <hardware/bluetooth.h>
#include <hardware/hardware.h>
#include <hardware/bt_gatt.h>
#include <hardware/ble_scanner.h>

//external/libchrome/base/
#include <base/message_loop/message_loop.h>
#include <base/run_loop.h>
#include <base/callback.h>
#include <base/logging.h>
#include "base/bind.h"

//#include "header.h" //mon header, où est définie l'action déclenchée à chaque récurrence (dans un autre fichier pour lisibilité)

#define LOG_TAG "bt_alrm"
#define KLOG_LEVEL 6

static int intervalle = 600; //secondes

static int eventct = 10;
static int epollfd;
static int wakealarm_fd;


bt_callbacks_t bt_callbacks = {
    sizeof(bt_callbacks_t),
	nullptr, nullptr, nullptr, nullptr, nullptr, nullptr, nullptr, nullptr, nullptr, nullptr, nullptr, nullptr, nullptr
};

void scan_result_cb(uint16_t event_type, uint8_t addr_type,
					 RawAddress *bda, uint8_t primary_phy,
					 uint8_t secondary_phy,
					 uint8_t advertising_sid, int8_t tx_power,
					 int8_t rssi, uint16_t periodic_adv_int,
					 std::vector<uint8_t> adv_data) {
		std::string addrstr = bda->ToString();
		//KLOG_WARNING(LOG_TAG, "VVNX scan result cb bdaddr=%s", addrstr.c_str());
		LOG(INFO) << "VVNX scan result cb bdaddr=" << addrstr.c_str() << ", rssi=-" << abs(rssi); //logcat -s bt_stack			 	
			
}

//hardware/ble_scanner.h
btgatt_scanner_callbacks_t btgatt_scanner_callbacks = {
	&scan_result_cb, //c'est un peu la seule cb qui m'interesse
    nullptr,
    nullptr,
    nullptr	
};

//hardware/bt_gatt.h
btgatt_callbacks_t bt_gatt_callbacks = {
	sizeof(btgatt_callbacks_t),
	nullptr,
    nullptr,
    &btgatt_scanner_callbacks
};


/* RegisterCallback --> scanner_id, status */
base::Callback<void(uint8_t, uint8_t)> registerCallback_vvnx = base::Bind([](uint8_t a, uint8_t b) { KLOG_WARNING(LOG_TAG, "register_cb scanner_id=%i status=%i\n",a, b);});

/* EnableCallback --> action, status */
base::Callback<void(uint8_t, uint8_t)> enableCallback_vvnx = base::Bind([](uint8_t a, uint8_t b) { KLOG_WARNING(LOG_TAG, "enable_cb action=%i status=%i\n",a, b);});


int main()
{
	int nevents = 0;

	struct epoll_event ev;
	struct epoll_event events[eventct];
	struct itimerspec itval;
	
	const hw_module_t* module;
	const bt_interface_t* hal_iface_;
	const bluetooth_device_t* hal_adapter_;
	
	/** epoll/timerfd **/

	epollfd = epoll_create(eventct);
	wakealarm_fd = timerfd_create(CLOCK_BOOTTIME_ALARM, TFD_NONBLOCK);
	
	KLOG_WARNING(LOG_TAG, "Creation du fd epoll=%i et du fd timerfd=%i\n", epollfd, wakealarm_fd);


	itval.it_value.tv_sec = 60; //le premier déclenchement
	itval.it_value.tv_nsec = 0;
	itval.it_interval.tv_sec = intervalle; //repeating
	itval.it_interval.tv_nsec = 0;
	
	ev.events = EPOLLIN | EPOLLWAKEUP;	
	ev.data.fd = wakealarm_fd;
	
	if (epoll_ctl(epollfd, EPOLL_CTL_ADD, wakealarm_fd, &ev) == -1) {
	KLOG_WARNING(LOG_TAG, "Plantade epollfd\n");
	return 1;
	}	
	
	if (timerfd_settime(wakealarm_fd, 0, &itval, NULL) != 0){
	KLOG_WARNING(LOG_TAG, "timerfd_settime() error\n");
	return 1;
	}
	
	
	sleep(30); //laisser le temps au bluetooth de se mettre en place
	
	/** bluetooth **/
	
	//loade libbluetooth.default.so
    int status = hal_util_load_bt_library(&module);
    if (status) {
      KLOG_WARNING(LOG_TAG, "Failed to load Bluetooth library \n");
      return 1;
    }
    
    hw_device_t* device;
    status = module->methods->open(module, BT_HARDWARE_MODULE_ID, &device);
    if (status) {
      KLOG_WARNING(LOG_TAG, "Failed to open the Bluetooth module \n");
      return 1;
    }
    
    hal_adapter_ = reinterpret_cast<bluetooth_device_t*>(device);
    hal_iface_ = hal_adapter_->get_bluetooth_interface();    

    status = hal_iface_->init(&bt_callbacks);
    if (status != BT_STATUS_SUCCESS) {
      KLOG_WARNING(LOG_TAG, "Failed to initialize Bluetooth stack \n");
      return 1;
    }    
    
    status = hal_iface_->enable(true); 
    if (status != BT_STATUS_SUCCESS) {
      KLOG_WARNING(LOG_TAG, "Failed to enable \n");
      return 1;
    }
    
    const btgatt_interface_t* gatt_iface = reinterpret_cast<const btgatt_interface_t*>(hal_iface_->get_profile_interface(BT_PROFILE_GATT_ID));  
      
    status = gatt_iface->init(&bt_gatt_callbacks);
    if (status != BT_STATUS_SUCCESS) {
      KLOG_WARNING(LOG_TAG, "Failed to initialize gatt \n");
      return 1;
    } 
	
	
	sleep(1); //obligatoire (faut laisser le temps au hardware de s'allumer?)
	

    BleScannerInterface* ble_iface = reinterpret_cast<BleScannerInterface*>(gatt_iface->scanner);    
    ble_iface->RegisterScanner(registerCallback_vvnx);   
  
 
		
	/**loop epoll_wait()/read()**/
	while (1) {		
	nevents = epoll_wait(epollfd, events, eventct, -1); 
		for (int n = 0; n < nevents; ++n) {			
			if (events[n].data.fd == wakealarm_fd) {				
				unsigned long long wakeups;				
				if (read(wakealarm_fd, &wakeups, sizeof(wakeups)) == -1) {
					KLOG_WARNING(LOG_TAG, "wakealarm_event: read wakealarm fd failed\n");
					return -1;
				}
				//c'est là que tu mets le code à déclencher à chaque occurence
				
				ble_iface->Scan(true);
			}			
		}    
	}
	
	
	
	//error: code will never be executed [-Werror,-Wunreachable-code]
    //_exit(0);
}
